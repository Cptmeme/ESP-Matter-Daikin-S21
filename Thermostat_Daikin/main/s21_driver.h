#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "daikin_s21.h"

// Represents the state of the AC
typedef struct {
    bool power;
    uint8_t mode;        // Uses FAIKIN_MODE_* enums
    float target_temp;   // Celsius
    float current_temp;  // Celsius (Room temp)
    float outside_temp;  // Celsius
    uint8_t fan_speed;   // Uses FAIKIN_FAN_* enums
    bool powerful;
} ac_state_t;

// Callback function type
typedef void (*s21_state_change_cb_t)(const ac_state_t *state);

class DaikinS21 {
public:
    DaikinS21();

    /**
     * @brief Initialize S21 Interface
     * @param tx_pin GPIO for TX
     * @param rx_pin GPIO for RX
     */
    esp_err_t Init(int tx_pin, int rx_pin);

    /**
     * @brief Main polling function. Call this periodically.
     */
    void Poll();

    // Setters
    void SetPower(bool on);
    void SetMode(uint8_t mode);
    void SetTemp(float temp);
    void SetFan(uint8_t fan);
    void SetPowerful(bool on);

    // Register a callback to update Matter attributes when AC changes
    void SetStateCallback(s21_state_change_cb_t cb);

    // Get current known state
    ac_state_t GetState() const { return m_state; }

private:
    ac_state_t m_state;
    bool m_dirty;
    bool m_powerful_dirty;
    s21_state_change_cb_t m_callback;

    // Internal helpers
    esp_err_t SendPacket(uint8_t cmd1, uint8_t cmd2, uint8_t *payload, int len);
    void ParseStatusG1(uint8_t *payload, int len);
    void ParseSettingsG6(uint8_t *payload, int len);
    void ParseSensorsGH(uint8_t *payload, int len);
    void ParseSensorsG9(uint8_t *payload, int len);
    void ParseSensorsSH(uint8_t *payload, int len); // <--- ADD THIS LINE
    void SendControlD1();
    void SendControlD6();
};