#pragma once

#include <stdint.h>
#include <stdbool.h>

// Protocol-neutral representation of the AC state. Every protocol backend
// (S21, CN_WIRED, X50A, ...) decodes into / encodes from this struct, and the
// Matter layer only ever sees this. Mode/fan use the FAIKIN_* enums from
// faikin_enums.h so the same values work across all protocols.
typedef struct {
    bool power;
    uint8_t mode;        // FAIKIN_MODE_*
    float target_temp;   // Celsius
    float current_temp;  // Celsius (room/home temp)
    float outside_temp;  // Celsius
    uint8_t fan_speed;   // FAIKIN_FAN_*
    bool powerful;

    // Energy monitoring. S21-only for now (FX60/FM/FU04); -1 = unknown / the
    // unit doesn't report it. Other backends leave these at -1.
    int32_t power_w;         // instantaneous power draw, W
    int32_t energy_total_wh; // lifetime total/outside energy, Wh
    int32_t energy_cool_wh;  // lifetime cooling energy, Wh
    int32_t energy_heat_wh;  // lifetime heating energy, Wh
} ac_state_t;

// Called by a backend whenever the AC state changes (controller-side or
// reported by the unit). Runs on the backend's poll task.
typedef void (*s21_state_change_cb_t)(const ac_state_t *state);
