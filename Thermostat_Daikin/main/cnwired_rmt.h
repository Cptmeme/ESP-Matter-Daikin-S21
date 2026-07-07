#pragma once
// Low-level CN_WIRED line driver (RMT-based pulse protocol).
// Ported from RevK's ESP32-Faikout cn_wired_driver.c, with the RevK framework
// (revk.h / jo_t JSON logging) dependencies removed so it builds standalone.

#include <driver/gpio.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Install the RMT TX/RX channels on the given pins. rx_invert/tx_invert apply
// GPIO inversion (needed when the line goes through an inverting level shifter,
// as on the S21 front-end).
esp_err_t cn_wired_driver_install(gpio_num_t rx_num, gpio_num_t tx_num, int rx_invert, int tx_invert);
void cn_wired_driver_delete(void);

// Read one CNW_PKT_LEN-byte packet. `wait` is a millisecond budget. Returns
// ESP_OK with the packet in rx[], ESP_ERR_TIMEOUT if nothing arrived, or
// ESP_ERR_INVALID_RESPONSE if the framing was malformed.
esp_err_t cn_wired_read_bytes(uint8_t *rx, int wait);

// Transmit one CNW_PKT_LEN-byte packet (caller fills checksum).
esp_err_t cn_wired_write_bytes(const uint8_t *buf);

#ifdef __cplusplus
}
#endif
