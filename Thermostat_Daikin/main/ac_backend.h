#pragma once

#include "esp_err.h"
#include "ac_state.h"

// Abstract interface every AC protocol driver implements. The ACManager owns
// one instance per supported protocol, probes each with Detect() at boot, and
// then drives whichever one the connected unit answers on.
//
// All backends share the same two GPIOs (TX1/RX1). Because only one can own the
// pins/peripherals at a time, the manager always pairs Init()+Detect() and, on
// failure, calls Deinit() before trying the next backend.
class ACBackend {
public:
    virtual ~ACBackend() {}

    // Human-readable protocol name, e.g. "S21". Used in logs.
    virtual const char *Name() const = 0;

    // Claim the GPIOs / peripherals this backend needs. Safe to call again
    // after a matching Deinit().
    virtual esp_err_t Init(int tx_pin, int rx_pin) = 0;

    // Release whatever Init() claimed, so another backend can try the pins.
    virtual void Deinit() {}

    // Probe for an AC speaking this protocol. Returns true if a valid response
    // was seen. Must be bounded in time (use read timeouts).
    virtual bool Detect() = 0;

    // True once this backend has an established, talking link.
    virtual bool IsConnected() const = 0;

    // Periodic service: poll the unit, push any state changes via the callback.
    virtual void Poll() = 0;

    // Control setters (no-op / buffered until connected, per backend).
    virtual void SetPower(bool on) = 0;
    virtual void SetMode(uint8_t mode) = 0;
    virtual void SetTemp(float temp) = 0;
    virtual void SetFan(uint8_t fan) = 0;
    virtual void SetPowerful(bool on) = 0;

    virtual ac_state_t GetState() const = 0;

    virtual void SetStateCallback(s21_state_change_cb_t cb) = 0;
};
