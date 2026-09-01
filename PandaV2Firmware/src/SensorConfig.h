#pragma once
#include <Arduino.h>

// PT sensor channel mapping and conversion for PandaV2.
//
// Mux A (16ch hardware, 2 sampled) → ADC1 CH0: PTs through INA132 difference amps.
// All other sensor types (LC, TC, current, power, board temp) have been
// removed in the PT-only lobotomy. If you re-add them, restore the mux B/C
// banks, the INA230 power monitors, and the TC cold-junction logic.

struct SensorCal {
    float scale;    // engineering_units = (voltage - offset) * scale
    float offset;   // voltage offset
};

static constexpr uint8_t NUM_PT_CH = 2;

// ── PT calibration ─────────────────────────────────────────────────
// Default: report raw voltage (scale=1, offset=0). Replace with
// per-channel cal from transducer datasheets once known.
static constexpr SensorCal PT_DEFAULT = {1.0f, 0.0f};

// ── Conversion ─────────────────────────────────────────────────────
inline float convertPT(float voltage, uint8_t ch) {
    (void)ch; // per-channel cal not yet implemented
    return (voltage - PT_DEFAULT.offset) * PT_DEFAULT.scale;
}
