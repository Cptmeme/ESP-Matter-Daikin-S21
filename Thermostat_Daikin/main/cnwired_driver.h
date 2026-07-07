#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "ac_state.h"
#include "ac_backend.h"

// CN_WIRED protocol backend (RMT pulse protocol, JST-PH connector).
//
// EXPERIMENTAL / UNVERIFIED on this board. CN_WIRED is electrically different
// from S21 (single-wire pulse, needs a data-line pull-up) and goes through the
// S21 inverting level shifter here, so the invert flags and timing must be
// confirmed against a real CN_WIRED unit before relying on this.
//
// CN_WIRED is a mostly passive protocol: the indoor unit broadcasts sensor
// reports periodically, and only emits full mode/state when a remote is used.
class DaikinCNWired : public ACBackend {
public:
    DaikinCNWired();

    const char *Name() const override { return "CN_WIRED"; }
    esp_err_t Init(int tx_pin, int rx_pin) override;
    void Deinit() override;
    bool Detect() override;
    bool IsConnected() const override { return m_connected; }
    void Poll() override;

    void SetPower(bool on) override;
    void SetMode(uint8_t mode) override;
    void SetTemp(float temp) override;
    void SetFan(uint8_t fan) override;
    void SetPowerful(bool on) override;

    ac_state_t GetState() const override { return m_state; }
    void SetStateCallback(s21_state_change_cb_t cb) override { m_callback = cb; }

private:
    bool ReadAndParse(int wait_ms);   // read one packet, update m_state; true if valid
    void SendModes();                 // push m_state to the unit

    ac_state_t m_state;
    bool m_connected;
    bool m_dirty;
    int m_tx_pin;
    int m_rx_pin;
    s21_state_change_cb_t m_callback;
};
