#pragma once

#include "esp_err.h"
#include "ac_state.h"
#include "ac_backend.h"
#include "faikin_enums.h"

#define AC_MAX_BACKENDS 4

// Owns every registered protocol backend, auto-detects which one the connected
// AC speaks (one probe per Poll cycle so the task never blocks for long), then
// forwards all control/state calls to the detected backend. Exposes the same
// method surface DaikinS21 used to, so the Matter glue in app_driver.cpp is
// unchanged apart from swapping the object type.
class ACManager {
public:
    ACManager();

    // Register a backend instance (not owned). Probe order = registration order.
    void AddBackend(ACBackend *b);

    esp_err_t Init(int tx_pin, int rx_pin);   // remembers pins; detection runs in Poll()
    void Poll();

    bool IsConnected() const;
    const char *ActiveName() const;

    void SetPower(bool on);
    void SetMode(uint8_t mode);
    void SetTemp(float temp);
    void SetFan(uint8_t fan);
    void SetPowerful(bool on);

    ac_state_t GetState() const;
    void SetStateCallback(s21_state_change_cb_t cb);

private:
    ACBackend *m_backends[AC_MAX_BACKENDS];
    int m_count;
    int m_tx_pin;
    int m_rx_pin;
    ACBackend *m_active;
    int m_next_probe;
    s21_state_change_cb_t m_callback;
    ac_state_t m_default_state;
};
