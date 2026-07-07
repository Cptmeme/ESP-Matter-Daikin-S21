#include "x50a_driver.h"
#include "faikin_enums.h"

#include <string.h>
#include <math.h>
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "X50A";

#define X50A_UART        UART_NUM_1
#define X50A_BAUD        9600
#define X50A_READ_TMO_MS 200
// The S21 front-end inverts both lines; X50A rides the same shifters.
#define X50A_INVERT_LINES 1

DaikinX50A::DaikinX50A()
    : m_connected(false), m_dirty(false), m_uart_num(X50A_UART), m_callback(nullptr) {
    m_state.power = false;
    m_state.mode = FAIKIN_MODE_AUTO;
    m_state.target_temp = 22.0f;
    m_state.current_temp = 21.0f;
    m_state.outside_temp = 0.0f;
    m_state.fan_speed = FAIKIN_FAN_AUTO;
    m_state.powerful = false;
    m_state.power_w = -1;
    m_state.energy_total_wh = -1;
    m_state.energy_cool_wh = -1;
    m_state.energy_heat_wh = -1;
}

esp_err_t DaikinX50A::Init(int tx_pin, int rx_pin) {
    m_connected = false;

    uart_config_t cfg = {};
    cfg.baud_rate = X50A_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_EVEN;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_param_config(X50A_UART, &cfg);
    if (!err) err = uart_set_pin(X50A_UART, tx_pin, rx_pin, -1, -1);
    if (!err) err = gpio_pullup_en((gpio_num_t)rx_pin);
#if X50A_INVERT_LINES
    if (!err) err = uart_set_line_inverse(X50A_UART, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);
#endif
    if (!err && !uart_is_driver_installed(X50A_UART))
        err = uart_driver_install(X50A_UART, 1024, 0, 0, NULL, 0);
    if (!err) err = uart_set_rx_full_threshold(X50A_UART, 1);
    if (!err) { vTaskDelay(pdMS_TO_TICKS(50)); uart_flush(X50A_UART); }
    if (err) ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(err));
    return err;
}

void DaikinX50A::Deinit() {
    if (uart_is_driver_installed(X50A_UART))
        uart_driver_delete(X50A_UART);
    m_connected = false;
}

int DaikinX50A::Command(uint8_t cmd, int txlen, const uint8_t *payload, uint8_t *rx_payload) {
    uint8_t buf[256];
    buf[0] = 0x06;
    buf[1] = cmd;
    buf[2] = txlen + 6;
    buf[3] = 1;
    buf[4] = 0;
    if (txlen > 0 && payload)
        memcpy(buf + 5, payload, txlen);
    uint8_t c = 0;
    for (int i = 0; i < 5 + txlen; i++)
        c += buf[i];
    buf[5 + txlen] = ~c;

    uart_flush_input(X50A_UART);
    uart_write_bytes(X50A_UART, buf, 6 + txlen);

    int rxlen = uart_read_bytes(X50A_UART, buf, sizeof(buf), pdMS_TO_TICKS(X50A_READ_TMO_MS));
    if (rxlen <= 0)
        return -1;

    // Checksum: sum of all bytes must be 0xFF.
    c = 0;
    for (int i = 0; i < rxlen; i++)
        c += buf[i];
    if (c != 0xFF)
        return -1;

    // Header sanity: 06 <cmd> <rxlen> 01 <nonzero>
    if (rxlen < 6 || buf[0] != 0x06 || buf[1] != cmd || buf[2] != rxlen || buf[3] != 1 || !buf[4])
        return -1;

    int paylen = rxlen - 6;
    if (rx_payload && paylen > 0)
        memcpy(rx_payload, buf + 5, paylen);
    return paylen;
}

bool DaikinX50A::Detect() {
    // X50A init handshake: 0xAA {0x01}. A valid framed reply means it's X50A.
    uint8_t init_payload[1] = { 0x01 };
    uint8_t rx[256];
    if (Command(0xAA, 1, init_payload, rx) < 0)
        return false;
    Command(0xBA, 0, NULL, rx);   // model (ignored)
    Command(0xBB, 0, NULL, rx);
    m_connected = true;
    return true;
}

void DaikinX50A::ParseStatusCA(const uint8_t *p, int len) {
    if (len < 7) return;
    m_state.power = (p[0] & 0x01);
    uint8_t mode = p[1] & 0x0F;   // low nibble == FAIKIN mode
    if (mode == FAIKIN_MODE_FAN || mode == FAIKIN_MODE_HEAT || mode == FAIKIN_MODE_COOL ||
        mode == FAIKIN_MODE_AUTO || mode == FAIKIN_MODE_DRY)
        m_state.mode = mode;
    m_state.fan_speed = (p[6] >> 4) & 0x07;
}

void DaikinX50A::ParseTempsBD(const uint8_t *p, int len) {
    if (len < 29) return;
    float room   = (int16_t)(p[2] | (p[3] << 8)) / 128.0f;
    float target = (int16_t)(p[8] | (p[9] << 8)) / 128.0f;
    if (room > 0.0f && room < 100.0f)   m_state.current_temp = room;
    if (target > 0.0f && target < 100.0f) m_state.target_temp = target;
}

void DaikinX50A::NotifyIfChanged(const ac_state_t &prev) {
    if (m_callback &&
        (prev.power != m_state.power || prev.mode != m_state.mode ||
         fabsf(prev.target_temp - m_state.target_temp) > 0.05f ||
         fabsf(prev.current_temp - m_state.current_temp) > 0.05f ||
         prev.fan_speed != m_state.fan_speed)) {
        m_callback(&m_state);
    }
}

void DaikinX50A::Poll() {
    ac_state_t prev = m_state;
    uint8_t rx[256];

    // 0xCA carries control when dirty (and reads status either way).
    uint8_t ca[17] = { 0 };
    uint8_t cb[2] = { 0 };
    if (m_dirty) {
        ca[0] = 2 + (m_state.power ? 1 : 0);
        ca[1] = 0x10 + m_state.mode;
        if (m_state.mode >= 1 && m_state.mode <= 3) {
            int t = lroundf(m_state.target_temp * 10);
            ca[3] = t / 10;
            ca[4] = 0x80 + (t % 10);
        }
        cb[0] = (m_state.mode == FAIKIN_MODE_HEAT || m_state.mode == FAIKIN_MODE_COOL) ? m_state.mode : 6;
        cb[1] = 0x80 + ((m_state.fan_speed & 7) << 4);
    }

    int n = Command(0xCA, sizeof(ca), ca, rx);
    if (n < 0) { m_connected = false; return; }
    ParseStatusCA(rx, n);

    if (m_dirty) {
        Command(0xCB, sizeof(cb), cb, rx);
        m_dirty = false;
    }

    n = Command(0xBD, 0, NULL, rx);
    if (n >= 0) ParseTempsBD(rx, n);

    m_connected = true;
    NotifyIfChanged(prev);
}

void DaikinX50A::SetPower(bool on)     { if (m_state.power != on)     { m_state.power = on;      m_dirty = true; } }
void DaikinX50A::SetMode(uint8_t mode) { if (m_state.mode != mode)    { m_state.mode = mode;     m_dirty = true; } }
void DaikinX50A::SetTemp(float temp)   { if (fabsf(m_state.target_temp - temp) > 0.05f) { m_state.target_temp = temp; m_dirty = true; } }
void DaikinX50A::SetFan(uint8_t fan)   { if (m_state.fan_speed != fan){ m_state.fan_speed = fan; m_dirty = true; } }
void DaikinX50A::SetPowerful(bool on)  { /* X50A powerful not modeled yet */ (void)on; }
