#include "cnwired_driver.h"
#include "cnwired_rmt.h"
#include "cn_wired.h"
#include "faikin_enums.h"

#include <esp_log.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "CN_WIRED_DRV";

// The S21 front-end inverts the line; CN_WIRED through it needs inversion too.
// Confirm against real hardware (a JST-PH CN_WIRED port + data-line pull-up).
#define CNWIRED_RX_INVERT 1
#define CNWIRED_TX_INVERT 1

// How long to listen for a broadcast before deciding it isn't CN_WIRED.
// (Faikin uses 5 s for a normal CN_WIRED read.) The wait now yields, so this
// no longer blocks the idle task / trips the task watchdog.
#define CNWIRED_DETECT_MS 5000
// Per-poll listen budget once connected (the unit broadcasts ~once/second).
#define CNWIRED_POLL_MS   3000

// ---- FAIKIN <-> CN_WIRED mode/fan mapping ----
// (Upstream's cnw_encode/decode helpers aren't in the shared header, so the
//  mappings live here, derived from the CNW_* constants in cn_wired.h.)

static int8_t cnw_decode_mode(const uint8_t *payload) {
    switch (payload[CNW_MODE_OFFSET] & CNW_MODE_MASK) {
        case CNW_DRY:  return FAIKIN_MODE_DRY;
        case CNW_FAN:  return FAIKIN_MODE_FAN;
        case CNW_COOL: return FAIKIN_MODE_COOL;
        case CNW_HEAT: return FAIKIN_MODE_HEAT;
        case CNW_AUTO: return FAIKIN_MODE_AUTO;
        default:       return FAIKIN_MODE_INVALID;
    }
}

static uint8_t cnw_encode_mode(uint8_t faikin_mode, bool power) {
    uint8_t m;
    switch (faikin_mode) {
        case FAIKIN_MODE_DRY:  m = CNW_DRY;  break;
        case FAIKIN_MODE_FAN:  m = CNW_FAN;  break;
        case FAIKIN_MODE_COOL: m = CNW_COOL; break;
        case FAIKIN_MODE_HEAT: m = CNW_HEAT; break;
        default:               m = CNW_AUTO; break;
    }
    if (!power) m |= CNW_MODE_POWEROFF;
    return m;
}

static int8_t cnw_decode_fan(const uint8_t *packet) {
    switch (packet[CNW_FAN_OFFSET]) {
        case CNW_FAN_AUTO:  return FAIKIN_FAN_AUTO;
        case CNW_FAN_QUIET: return FAIKIN_FAN_QUIET;
        case CNW_FAN_1:     return FAIKIN_FAN_1;
        case CNW_FAN_2:     return FAIKIN_FAN_2;
        case CNW_FAN_3:     return FAIKIN_FAN_3;
        default:            return FAIKIN_FAN_INVALID; // e.g. CNW_FAN_POWERFUL
    }
}

static uint8_t cnw_encode_fan(uint8_t faikin_fan) {
    switch (faikin_fan) {
        case FAIKIN_FAN_QUIET: return CNW_FAN_QUIET;
        case FAIKIN_FAN_1:     return CNW_FAN_1;
        case FAIKIN_FAN_2:     return CNW_FAN_2;
        case FAIKIN_FAN_3:
        case FAIKIN_FAN_4:
        case FAIKIN_FAN_5:     return CNW_FAN_3;
        default:               return CNW_FAN_AUTO;
    }
}

DaikinCNWired::DaikinCNWired()
    : m_connected(false), m_dirty(false), m_tx_pin(-1), m_rx_pin(-1), m_callback(nullptr) {
    m_state.power = false;
    m_state.mode = FAIKIN_MODE_AUTO;
    m_state.target_temp = 20.0f;
    m_state.current_temp = 20.0f;
    m_state.outside_temp = 0.0f;
    m_state.fan_speed = FAIKIN_FAN_AUTO;
    m_state.powerful = false;
    m_state.power_w = -1;
    m_state.energy_total_wh = -1;
    m_state.energy_cool_wh = -1;
    m_state.energy_heat_wh = -1;
}

esp_err_t DaikinCNWired::Init(int tx_pin, int rx_pin) {
    m_tx_pin = tx_pin;
    m_rx_pin = rx_pin;
    m_connected = false;
    return cn_wired_driver_install((gpio_num_t)rx_pin, (gpio_num_t)tx_pin,
                                   CNWIRED_RX_INVERT, CNWIRED_TX_INVERT);
}

void DaikinCNWired::Deinit() {
    cn_wired_driver_delete();
    m_connected = false;
}

bool DaikinCNWired::ReadAndParse(int wait_ms) {
    uint8_t pkt[CNW_PKT_LEN];
    esp_err_t err = cn_wired_read_bytes(pkt, wait_ms);
    if (err != ESP_OK) {
        return false;
    }
    if (cnw_checksum(pkt) != pkt[CNW_CRC_TYPE_OFFSET]) {
        ESP_LOGW(TAG, "CN_WIRED bad checksum");
        return false;
    }

    ac_state_t prev = m_state;
    switch (pkt[CNW_CRC_TYPE_OFFSET] & CNW_TYPE_MASK) {
        case CNW_SENSOR_REPORT:
            // Periodic room-temperature broadcast (BCD).
            m_state.current_temp = decode_bcd(pkt[CNW_TEMP_OFFSET]);
            break;
        case CNW_MODE_CHANGED: {
            m_state.power = !(pkt[CNW_MODE_OFFSET] & CNW_MODE_POWEROFF);
            int8_t mode = cnw_decode_mode(pkt);
            if (mode != FAIKIN_MODE_INVALID) m_state.mode = mode;
            m_state.target_temp = decode_bcd(pkt[CNW_TEMP_OFFSET]);
            int8_t fan = cnw_decode_fan(pkt);
            if (fan != FAIKIN_FAN_INVALID) m_state.fan_speed = fan;
            m_state.powerful = (pkt[CNW_FAN_OFFSET] == CNW_FAN_POWERFUL);
            break;
        }
        default:
            ESP_LOGD(TAG, "CN_WIRED unknown packet type 0x%02X", pkt[CNW_CRC_TYPE_OFFSET]);
            break;
    }

    m_connected = true;
    // Notify on any change.
    if (m_callback &&
        (prev.power != m_state.power || prev.mode != m_state.mode ||
         fabsf(prev.target_temp - m_state.target_temp) > 0.05f ||
         fabsf(prev.current_temp - m_state.current_temp) > 0.05f ||
         prev.fan_speed != m_state.fan_speed || prev.powerful != m_state.powerful)) {
        m_callback(&m_state);
    }
    return true;
}

void DaikinCNWired::SendModes() {
    uint8_t buf[CNW_PKT_LEN];
    uint8_t fan = m_state.powerful ? CNW_FAN_POWERFUL : cnw_encode_fan(m_state.fan_speed);

    buf[CNW_TEMP_OFFSET] = encode_bcd((uint8_t)lroundf(m_state.target_temp));
    buf[1] = 0x04;   // fixed, known-working values (upstream)
    buf[2] = 0x50;
    buf[CNW_MODE_OFFSET] = cnw_encode_mode(m_state.mode, m_state.power);
    buf[CNW_FAN_OFFSET] = fan;
    buf[CNW_SPECIALS_OFFSET] = 0;
    buf[6] = 0x10;
    buf[CNW_CRC_TYPE_OFFSET] = CNW_COMMAND;
    buf[CNW_CRC_TYPE_OFFSET] = cnw_checksum(buf);

    if (cn_wired_write_bytes(buf) == ESP_OK) {
        m_dirty = false;
    }
}

bool DaikinCNWired::Detect() {
    // Listen for one valid broadcast. CN_WIRED only does 1C temp steps and is
    // passive, so detection is just "did we hear a well-formed packet?".
    return ReadAndParse(CNWIRED_DETECT_MS);
}

void DaikinCNWired::Poll() {
    // Receive whatever the unit broadcast this cycle.
    ReadAndParse(CNWIRED_POLL_MS);
    // CN_WIRED devices expect the controller to echo modes back; send after rx.
    if (m_dirty) {
        SendModes();
    }
}

// CN_WIRED only supports whole-degree setpoints.
void DaikinCNWired::SetPower(bool on)      { if (m_state.power != on) { m_state.power = on; m_dirty = true; } }
void DaikinCNWired::SetMode(uint8_t mode)  { if (m_state.mode != mode) { m_state.mode = mode; m_dirty = true; } }
void DaikinCNWired::SetTemp(float temp)    { float r = roundf(temp); if (fabsf(m_state.target_temp - r) > 0.05f) { m_state.target_temp = r; m_dirty = true; } }
void DaikinCNWired::SetFan(uint8_t fan)    { if (m_state.fan_speed != fan) { m_state.fan_speed = fan; m_dirty = true; } }
void DaikinCNWired::SetPowerful(bool on)   { if (m_state.powerful != on) { m_state.powerful = on; m_dirty = true; } }
