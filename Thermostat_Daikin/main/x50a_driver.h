#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "ac_state.h"
#include "ac_backend.h"

// Daikin X50A protocol backend (the "newer" binary protocol on the X50A/X35A
// connector). Hardware UART, 9600 8E1, line-inverted for the S21 level shifter.
// Frame: 06 <cmd> <len> 01 00 <payload...> <~checksum>.
//
// EXPERIMENTAL / UNVERIFIED on this board. Electrically X50A is the same 2-wire
// UART front-end as S21 (so this board should drive it), but it has never been
// tested here against a real X50A unit. Mode/fan/temperature decoding follows
// RevK's Faikout; confirm against hardware before relying on it.
class DaikinX50A : public ACBackend {
public:
    DaikinX50A();

    const char *Name() const override { return "X50A"; }
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
    // Full command/response exchange. Returns payload length (>=0) on success
    // with the payload copied to rx_payload, or -1 on timeout/framing error.
    int Command(uint8_t cmd, int txlen, const uint8_t *payload, uint8_t *rx_payload);
    void ParseStatusCA(const uint8_t *payload, int len);
    void ParseTempsBD(const uint8_t *payload, int len);
    void NotifyIfChanged(const ac_state_t &prev);

    ac_state_t m_state;
    bool m_connected;
    bool m_dirty;
    int m_uart_num;
    s21_state_change_cb_t m_callback;
};
