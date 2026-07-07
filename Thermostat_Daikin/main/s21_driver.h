#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "daikin_s21.h"
#include "ac_state.h"
#include "ac_backend.h"

// Daikin S21 protocol backend: bit-banged software UART, 2400 baud 8E2,
// inverted through the BSS138 level shifters. See s21_driver.cpp.
class DaikinS21 : public ACBackend {
public:
    DaikinS21();

    // --- ACBackend ---
    const char *Name() const override { return "S21"; }
    esp_err_t Init(int tx_pin, int rx_pin) override;
    void Deinit() override;
    bool Detect() override;
    bool IsConnected() const override;
    void Poll() override;

    void SetPower(bool on) override;
    void SetMode(uint8_t mode) override;
    void SetTemp(float temp) override;
    void SetFan(uint8_t fan) override;
    void SetPowerful(bool on) override;

    ac_state_t GetState() const override { return m_state; }
    void SetStateCallback(s21_state_change_cb_t cb) override { m_callback = cb; }

private:
    ac_state_t m_state;
    bool m_dirty;
    bool m_powerful_dirty;
    s21_state_change_cb_t m_callback;
    bool m_energy_supported;   // set false if the unit NAKs the energy queries
    int m_energy_miss;         // consecutive cycles with no energy response

    // Internal helpers
    esp_err_t SendPacket(uint8_t cmd1, uint8_t cmd2, uint8_t *payload, int len);
    void ParseStatusG1(uint8_t *payload, int len);
    void ParseSettingsG6(uint8_t *payload, int len);
    void ParseSensorsGH(uint8_t *payload, int len);
    void ParseSensorsG9(uint8_t *payload, int len);
    void ParseSensorsSH(uint8_t *payload, int len);
    void ParsePowerGX(uint8_t *payload, int len);   // FX60 -> instantaneous W
    void ParseEnergyGM(uint8_t *payload, int len);   // FM   -> total Wh
    void ParseEnergyGU(uint8_t *payload, int len);   // FU04 -> cool/heat Wh
    void SendControlD1();
    void SendControlD6();
};
