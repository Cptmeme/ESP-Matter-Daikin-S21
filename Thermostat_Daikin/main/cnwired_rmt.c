// CN_WIRED RMT line driver, ported from RevK ESP32-Faikout (cn_wired_driver.c).
// Timing and framing are kept identical to upstream; only the RevK framework
// dependencies (revk.h, jo_t logging, the `cnmark900` setting) were removed.
//
// NOTE: untested against real CN_WIRED hardware on this board. The S21 front-end
// inverts the line, so it is installed with rx/tx inversion (see cnwired_driver).

#include <string.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "cn_wired.h"
#include "cnwired_rmt.h"

static const char TAG[] = "CN_WIRED";

#define CN_WIRED_SYNC   2600    // uS
#define CN_WIRED_START  1000    // uS  (upstream: cnmark900 ? 900 : 1000)
#define CN_WIRED_SPACE  300     // uS
#define CN_WIRED_0      400     // uS
#define CN_WIRED_1      1000    // uS
#define CN_WIRED_IDLE   16000   // uS
#define CN_WIRED_TERM   2000    // uS
#define CN_WIRED_MARGIN 200     // uS

static rmt_channel_handle_t rmt_tx = NULL;
static rmt_channel_handle_t rmt_rx = NULL;
static rmt_encoder_handle_t rmt_encoder = NULL;
static rmt_symbol_word_t rmt_rx_raw[70];   // room for 66 symbols + a little slack
static volatile size_t rmt_rx_len = 0;

static const rmt_receive_config_t rmt_rx_config = {
    .signal_range_min_ns = 1000,                                   // ignore glitches
    .signal_range_max_ns = (CN_WIRED_SYNC + CN_WIRED_MARGIN) * 1000,
};

// See upstream comment: the RMT driver forces idle level to 0, so we invert the
// whole thing (and compensate with GPIO inversion) to get an idle-HIGH line.
// Hence these are intentionally swapped.
static const uint16_t TX_HIGH = 0;
static const uint16_t TX_LOW  = 1;

static const rmt_transmit_config_t rmt_tx_config = {
    .flags = { .eot_level = TX_HIGH },
};

static bool rmt_rx_callback(rmt_channel_handle_t channel,
                            const rmt_rx_done_event_data_t *edata, void *user_data)
{
    if (edata->num_symbols < 64) {
        // Too short — restart rx
        rmt_rx_len = 0;
        rmt_receive(rmt_rx, rmt_rx_raw, sizeof(rmt_rx_raw), &rmt_rx_config);
        return false;
    }
    rmt_rx_len = edata->num_symbols;
    return false;
}

esp_err_t cn_wired_driver_install(gpio_num_t rx_num, gpio_num_t tx_num, int rx_invert, int tx_invert)
{
    esp_err_t err = ESP_OK;

    if (rmt_rx || rmt_tx) {
        ESP_LOGE(TAG, "cn_wired driver already initialized");
        return ESP_FAIL;
    }

    if (!rmt_encoder) {
        rmt_copy_encoder_config_t encoder_config = {};
        err = rmt_new_copy_encoder(&encoder_config, &rmt_encoder);
    }
    if (!err) {
        rmt_tx_channel_config_t tx_chan_config = {
            .gpio_num = tx_num,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 1 * 1000 * 1000,   // 1 tick = 1 us
            .mem_block_symbols = 72,
            .trans_queue_depth = 1,
            .flags = { .invert_out = (uint32_t)(tx_invert ^ TX_LOW) },
        };
        err = rmt_new_tx_channel(&tx_chan_config, &rmt_tx);
        if (rmt_tx && !err)
            err = rmt_enable(rmt_tx);
    }
    if (!err) {
        rmt_rx_channel_config_t rx_chan_config = {
            .gpio_num = rx_num,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 1 * 1000 * 1000,
            .mem_block_symbols = 72,
            .flags = { .invert_in = (uint32_t)(rx_invert ? 1 : 0) },
        };
        err = rmt_new_rx_channel(&rx_chan_config, &rmt_rx);
        if (rmt_rx && !err) {
            rmt_rx_event_callbacks_t cbs = { .on_recv_done = rmt_rx_callback };
            err = rmt_rx_register_event_callbacks(rmt_rx, &cbs, NULL);
            if (!err)
                err = rmt_enable(rmt_rx);
        }
    }
    rmt_rx_len = 0;
    if (!err)
        err = rmt_receive(rmt_rx, rmt_rx_raw, sizeof(rmt_rx_raw), &rmt_rx_config);
    return err;
}

void cn_wired_driver_delete(void)
{
    if (rmt_tx) {
        rmt_disable(rmt_tx);
        rmt_del_channel(rmt_tx);
        rmt_tx = NULL;
    }
    if (rmt_rx) {
        rmt_disable(rmt_rx);
        rmt_del_channel(rmt_rx);
        rmt_rx = NULL;
    }
}

esp_err_t cn_wired_read_bytes(uint8_t *rx, int wait)
{
    esp_err_t err = ESP_OK;
    const char *e = NULL;
    int p = 0, dur = 0;

    if (!rmt_tx || !rmt_encoder || !rmt_rx)
        return ESP_ERR_INVALID_STATE;

    // Wait for an rx frame, yielding to the scheduler (~10 ms granularity) so we
    // don't starve the idle task / trip the task watchdog during long listens.
    int waited = 0;
    while (!rmt_rx_len && waited < wait) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
    if (!rmt_rx_len)
        return ESP_ERR_TIMEOUT;

    // Sanity-check framing
    if (!e && rmt_rx_len != CNW_PKT_LEN * 8 + 2)
        e = "Wrong length";
    if (!e && rmt_rx_raw[p].level0)
        e = "Bad start polarity";
    if (!e && ((dur = rmt_rx_raw[p].duration0) < CN_WIRED_SYNC - CN_WIRED_MARGIN || dur > CN_WIRED_SYNC + CN_WIRED_MARGIN))
        e = "Bad start duration";
    if (!e && ((dur = rmt_rx_raw[p].duration1) < CN_WIRED_START - CN_WIRED_MARGIN || dur > CN_WIRED_START + CN_WIRED_MARGIN))
        e = "Bad start bit";
    p++;
    for (int i = 0; !e && i < CNW_PKT_LEN; i++) {
        rx[i] = 0;
        for (uint8_t b = 0x01; !e && b; b <<= 1) {
            if (!e && ((dur = rmt_rx_raw[p].duration0) < CN_WIRED_SPACE - CN_WIRED_MARGIN || dur > CN_WIRED_SPACE + CN_WIRED_MARGIN))
                e = "Bad space duration";
            if (!e && (dur = rmt_rx_raw[p].duration1) > CN_WIRED_1 - CN_WIRED_MARGIN && dur < CN_WIRED_1 + CN_WIRED_MARGIN)
                rx[i] |= b;
            else if (!e && ((dur = rmt_rx_raw[p].duration1) < CN_WIRED_0 - CN_WIRED_MARGIN || dur > CN_WIRED_1 + CN_WIRED_MARGIN))
                e = "Bad bit duration";
            p++;
        }
    }

    if (e) {
        ESP_LOGW(TAG, "rx framing error: %s (dur=%d)", e, dur);
        err = ESP_ERR_INVALID_RESPONSE;
    }

    // Arm next rx
    rmt_rx_len = 0;
    rmt_receive(rmt_rx, rmt_rx_raw, sizeof(rmt_rx_raw), &rmt_rx_config);
    return err;
}

esp_err_t cn_wired_write_bytes(const uint8_t *buf)
{
    esp_err_t err;
    rmt_symbol_word_t seq[3 + CNW_PKT_LEN * 8 + 1];
    int p = 0;

    // SYNC must be all LOW. RMT symbols come in HIGH/LOW pairs (even count), so
    // the odd-count sequence is balanced by splitting SYNC into two LOW halves.
    seq[p].duration0 = CN_WIRED_SYNC - 1000;
    seq[p].level0 = TX_LOW;
    seq[p].duration1 = 1000;
    seq[p++].level1 = TX_LOW;

    // local helper (C: nested via static-less loop instead of GCC nested fn)
    #define CNW_ADD(d) do { \
        seq[p].duration0 = (d); seq[p].level0 = TX_HIGH; \
        seq[p].duration1 = CN_WIRED_SPACE; seq[p++].level1 = TX_LOW; \
    } while (0)

    CNW_ADD(CN_WIRED_START);
    for (int i = 0; i < CNW_PKT_LEN; i++)
        for (uint8_t b = 0x01; b; b <<= 1)
            CNW_ADD((buf[i] & b) ? CN_WIRED_1 : CN_WIRED_0);

    seq[p].duration0 = CN_WIRED_IDLE;
    seq[p].level0 = TX_HIGH;
    seq[p].duration1 = CN_WIRED_TERM;
    seq[p++].level1 = TX_LOW;
    #undef CNW_ADD

    err = rmt_transmit(rmt_tx, rmt_encoder, seq, p * sizeof(rmt_symbol_word_t), &rmt_tx_config);
    if (!err)
        err = rmt_tx_wait_all_done(rmt_tx, 1000);
    return err;
}
