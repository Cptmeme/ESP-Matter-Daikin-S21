#include "s21_driver.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

static const char *TAG = "S21_DRIVER";
#define BIT_DELAY_US 417 
#define BUF_SIZE 256

static bool s_connected = false;
static int s_tx_pin = 0;
static int s_rx_pin = 0;
static portMUX_TYPE s_spinlock = portMUX_INITIALIZER_UNLOCKED;

static void sw_write_byte(uint8_t byte) {
    taskENTER_CRITICAL(&s_spinlock);
    gpio_set_level((gpio_num_t)s_tx_pin, 1);
    esp_rom_delay_us(BIT_DELAY_US);
    int ones = 0;
    for (int i = 0; i < 8; i++) {
        int bit = (byte >> i) & 0x01;
        if (bit) ones++;
        gpio_set_level((gpio_num_t)s_tx_pin, bit ? 0 : 1); 
        esp_rom_delay_us(BIT_DELAY_US);
    }
    int parity = (ones % 2 == 0) ? 0 : 1;
    gpio_set_level((gpio_num_t)s_tx_pin, parity ? 0 : 1); 
    esp_rom_delay_us(BIT_DELAY_US);
    gpio_set_level((gpio_num_t)s_tx_pin, 0);
    taskEXIT_CRITICAL(&s_spinlock);
    esp_rom_delay_us(BIT_DELAY_US * 2);
}

static int sw_read_byte(uint32_t timeout_ms) {
    int64_t start = esp_timer_get_time();
    int64_t timeout_us = timeout_ms * 1000;
    while (gpio_get_level((gpio_num_t)s_rx_pin) == 0) {
        if (esp_timer_get_time() - start > timeout_us) return -1;
    }
    esp_rom_delay_us(BIT_DELAY_US / 2);
    if (gpio_get_level((gpio_num_t)s_rx_pin) == 0) return -1;
    esp_rom_delay_us(BIT_DELAY_US);
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        int level = gpio_get_level((gpio_num_t)s_rx_pin);
        if (level == 0) byte |= (1 << i); 
        esp_rom_delay_us(BIT_DELAY_US);
    }
    esp_rom_delay_us(BIT_DELAY_US * 2); 
    return byte;
}

DaikinS21::DaikinS21() {
    m_dirty = false;
    m_powerful_dirty = false;
    m_callback = nullptr;
    m_state.power = false;
    m_state.mode = FAIKIN_MODE_AUTO;
    m_state.target_temp = 22.0;
    m_state.fan_speed = FAIKIN_FAN_AUTO;
    m_state.current_temp = 21.0;
    m_state.powerful = false;
}

esp_err_t DaikinS21::Init(int tx_pin, int rx_pin) {
    s_tx_pin = tx_pin;
    s_rx_pin = rx_pin;
    gpio_config_t tx_conf = {};
    tx_conf.pin_bit_mask = (1ULL << tx_pin);
    tx_conf.mode = GPIO_MODE_OUTPUT;
    tx_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&tx_conf));
    gpio_set_level((gpio_num_t)tx_pin, 0); 
    gpio_config_t rx_conf = {};
    rx_conf.pin_bit_mask = (1ULL << rx_pin);
    rx_conf.mode = GPIO_MODE_INPUT;
    rx_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&rx_conf));
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    return ESP_OK;
}

esp_err_t DaikinS21::SendPacket(uint8_t cmd1, uint8_t cmd2, uint8_t *payload, int len) {
    sw_write_byte(STX);
    sw_write_byte(cmd1);
    sw_write_byte(cmd2);
    uint8_t buf[64];
    buf[0] = STX; buf[1] = cmd1; buf[2] = cmd2;
    if (len > 0) memcpy(&buf[3], payload, len);
    for(int i=0; i<len; i++) sw_write_byte(payload[i]);
    uint8_t crc = s21_checksum(buf, len + 3 + 2); 
    sw_write_byte(crc);
    sw_write_byte(ETX);

    int ack = sw_read_byte(800);
    if (ack == -1) return ESP_ERR_TIMEOUT;
    if (ack == NAK) return ESP_FAIL;

    uint8_t rx_buf[BUF_SIZE];
    int rx_idx = 0;
    if (ack == STX) rx_buf[rx_idx++] = STX;
    else if (ack == ACK) {
        int val = sw_read_byte(500);
        if (val == STX) rx_buf[rx_idx++] = STX;
        else return ESP_OK; 
    }
    while (rx_idx < BUF_SIZE) {
        int val = sw_read_byte(100);
        if (val == -1) break;
        rx_buf[rx_idx++] = (uint8_t)val;
        if (val == ETX) break;
    }

    if (rx_idx > 3 && rx_buf[rx_idx-1] == ETX) {
        uint8_t calc_crc = s21_checksum(rx_buf, rx_idx);
        if (calc_crc == rx_buf[rx_idx-2]) {
            sw_write_byte(ACK);
            s_connected = true;
            
            // DEBUG DUMP (Keep this to verify mode byte)
            if (rx_buf[1] == 'G' && rx_buf[2] == '1') {
                printf("[S21] G1 RAW: ");
                for(int k=0; k<rx_idx; k++) printf("%02X ", rx_buf[k]);
                printf("\n");
            }

            if ((rx_buf[1] == 'G' || rx_buf[1] == 'H') && rx_buf[2] == '1') {
                 ParseStatusG1(&rx_buf[3], rx_idx - 5);
            }
            else if (rx_buf[1] == 'S' && rx_buf[2] == 'H') {
                 ParseSensorsSH(&rx_buf[3], rx_idx - 5);
            }
            else if (rx_buf[1] == 'G' && rx_buf[2] == '6') {
                 ParseSettingsG6(&rx_buf[3], rx_idx - 5);
            }
        }
    }
    return ESP_OK;
}

void DaikinS21::ParseStatusG1(uint8_t *payload, int len) {
    if (len < 4) return;
    
    // Decode Mode: Byte 1
    uint8_t raw_mode = payload[1];
    uint8_t mode = FAIKIN_MODE_AUTO;
    
    if (raw_mode == 0x33) mode = FAIKIN_MODE_COOL;      // '3'
    else if (raw_mode == 0x34) mode = FAIKIN_MODE_HEAT; // '4'
    else if (raw_mode == 0x32) mode = FAIKIN_MODE_DRY;  // '2'
    else if (raw_mode == 0x36) mode = FAIKIN_MODE_FAN;  // '6'
    
    bool pwr = (payload[0] == '1');
    float t = s21_decode_target_temp(payload[2]);

    // Check if something changed
    bool changed = (m_state.power != pwr || m_state.mode != mode || m_state.target_temp != t);
    
    if (changed) {
        ESP_LOGI(TAG, "Status Change Detected! Pwr:%d Mode:%d (Raw:%02X)", pwr, mode, raw_mode);
    }

    m_state.power = pwr;
    m_state.mode = mode;
    m_state.target_temp = t;

    if (changed && m_callback) m_callback(&m_state);
}

void DaikinS21::ParseSettingsG6(uint8_t *payload, int len) {
    if (len < 4) return;
    
    // Check the powerful bit (0x02) on the first payload byte
    bool pow = (payload[0] & S21_FLAG_POWERFUL) != 0;
    
    if (m_state.powerful != pow) {
        ESP_LOGI(TAG, "Powerful Mode Change Detected! Powerful:%d", pow);
        m_state.powerful = pow;
        if (m_callback) m_callback(&m_state);
    }
}

void DaikinS21::SendControlD6() {
    uint8_t payload[4];
    
    // Byte 0: Base '0' + Powerful (0x02) + Comfort (0x40) + Quiet (0x80)
    payload[0] = S21_D6_BASE_BYTE;
    if (m_state.powerful) payload[0] += S21_FLAG_POWERFUL;
    
    // Bytes 1-3: Default to '0' (Unless you implement streamer/sensor/led later)
    payload[1] = S21_D6_BASE_BYTE;
    payload[2] = S21_D6_BASE_BYTE;
    payload[3] = S21_D6_BASE_BYTE;

    SendPacket('D', '6', payload, 4);
    
    // Clear the dirty flag specific to the powerful state
    m_powerful_dirty = false; 
}

void DaikinS21::ParseSensorsSH(uint8_t *payload, int len) {
    float room = s21_decode_float_sensor(payload);
    if (room > 0.0 && room < 50.0) {
        if (fabs(m_state.current_temp - room) > 0.1) {
             ESP_LOGI(TAG, "Room Temp Update (SH): %.1f", room);
             m_state.current_temp = room;
             if (m_callback) m_callback(&m_state);
        }
    }
}

void DaikinS21::ParseSensorsGH(uint8_t *p, int l) {}
void DaikinS21::ParseSensorsG9(uint8_t *p, int l) {}

void DaikinS21::SendControlD1() {
    uint8_t payload[4];
    payload[0] = m_state.power ? '1' : '0';
    switch(m_state.mode) {
        case FAIKIN_MODE_COOL: payload[1] = 0x33; break;
        case FAIKIN_MODE_HEAT: payload[1] = 0x34; break;
        case FAIKIN_MODE_DRY:  payload[1] = 0x32; break;
        case FAIKIN_MODE_FAN:  payload[1] = 0x36; break;
        default: payload[1] = 0x30; break; 
    }
    if (m_state.mode == FAIKIN_MODE_FAN || m_state.mode == FAIKIN_MODE_DRY)
        payload[2] = AC_MIN_TEMP_VALUE;
    else
        payload[2] = (uint8_t)s21_encode_target_temp(m_state.target_temp);

    if (m_state.fan_speed == FAIKIN_FAN_AUTO) payload[3] = 'A'; 
    else payload[3] = '3' + (m_state.fan_speed - 1); 
    SendPacket('D', '1', payload, 4);
    m_dirty = false;
}

void DaikinS21::Poll() {
    if (!s_connected) {
        SendPacket('F', '8', NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        SendPacket('F', '1', NULL, 0); 
        return; 
    }
    if (m_dirty) {
        SendControlD1();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (m_powerful_dirty) {
        SendControlD6();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Poll Status (F1 -> G1)
    SendPacket('F', '1', NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    SendPacket('F', '6', NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Poll Sensor (RH -> SH)
    SendPacket('R', 'H', NULL, 0);
}

// Setters
void DaikinS21::SetPower(bool on) { if(m_state.power != on) { m_state.power = on; m_dirty = true; } }
void DaikinS21::SetMode(uint8_t mode) { if(m_state.mode != mode) { m_state.mode = mode; m_dirty = true; } }
void DaikinS21::SetTemp(float temp) { if(fabs(m_state.target_temp - temp) > 0.1) { m_state.target_temp = temp; m_dirty = true; } }
void DaikinS21::SetFan(uint8_t fan) { m_state.fan_speed = fan; m_dirty = true; }
void DaikinS21::SetStateCallback(s21_state_change_cb_t cb) { m_callback = cb; }
void DaikinS21::SetPowerful(bool on) { 
    if(m_state.powerful != on) { 
        m_state.powerful = on; 
        m_powerful_dirty = true;
    } 
}