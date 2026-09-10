#pragma once
#include <Arduino.h>
#include "hardware-configs/BoardConfig.hpp"

// =============================================================================
// Firmware-side bang-bang pressure controller (V1).
//
// One BBController instance per side (LOX, Fuel) runs autonomously on V1 once
// armed and enabled. Each instance:
//   - Reads a single PT channel from the ptData[] array fed by the V2 crossover.
//   - Drives a "press" DC solenoid while in SUSTAIN state (bang-bang control).
//   - Drives a "vent" DC solenoid while in AUTO_VENT or ABORT state.
//   - Transitions between states autonomously (autovent_trigger) or by GC
//     command (manual vent, manual abort).
//   - Optionally nudges its own setpoint within a configured window to chase a
//     target mass-flow, using venturi PT pairs (when wired in BoardConfig).
//
// Safety invariants:
//   - Always starts DISABLED on power-up, even if EEPROM config is present.
//   - Disarm → forceSafe(): press closed, vent closed, state = DISABLED,
//     abort-latch cleared. This is the only way to clear an ABORT latch.
//   - PT reading outside BB_PRESSURE_MIN_PSI..BB_PRESSURE_MAX_PSI while armed →
//     forceSafe() + SANITY_FAIL event. No self-recovery; operator must re-enable.
//   - Cannot enter AUTO_VENT or ABORT if vent DC channel is unset (BB_DC_CH_UNSET).
//     Command is rejected with AV_NO_HW event.
//   - Mass-flow correction only applies when BOTH venturi PT indices are set
//     AND mdot_enabled is true. Otherwise the setpoint is never modified.
//   - Every state transition, valve actuation, sanity violation, and config
//     write emits a structured EVT: line to the telemetry stream so GC has a
//     complete audit trail.
//
// EEPROM layout is magic + struct + CRC. Layout changes bump BB_EEPROM_MAGIC
// in BoardConfig.hpp — old blobs are ignored, not migrated.
// =============================================================================

// ── Persisted config (one struct per side) ────────────────────────────────
struct BBConfig {
    // Core bang-bang
    float    setpoint_psi      = 200.0f;
    float    deadband_psi      = 10.0f;   // symmetric, ±deadband/2 around setpoint
    uint32_t wait_ms           = 500;     // minimum closed dwell before reopening
    uint32_t max_open_ms       = 0;       // slow-press cap; 0 disables
    uint32_t close_delay_ms    = 15;      // energized→mechanically closed delay

    // Auto-venting
    float    autovent_trigger  = 100000.0f; // press > this enters AUTO_VENT; huge sentinel = disabled
    bool     autovent_enabled  = false;     // when false, auto-trigger is ignored (manual still works)

    // Mass-flow correction (only active if venturi PTs are wired AND enabled)
    float    mdot_target       = 0.0f;    // target mass flow [kg/s]
    float    sp_min            = 0.0f;    // setpoint lower bound during correction [psi]
    float    sp_max            = 0.0f;    // setpoint upper bound during correction [psi]
    float    mdot_gain         = 0.0f;    // psi per (kg/s error) per update tick
    float    density_kgm3      = 0.0f;    // propellant density for venturi calc; <=0 disables
    bool     mdot_enabled      = false;
};

struct BBEepromBlock {
    uint16_t magic;
    BBConfig lox;
    BBConfig fuel;
    uint8_t  crc;
};

// ── State machine ─────────────────────────────────────────────────────────
enum class BBState : uint8_t {
    DISABLED  = 0,  // no valves driven by BB; manual control allowed
    SUSTAIN   = 1,  // bang-bang controlling press valve against setpoint
    AUTO_VENT = 2,  // press closed, vent open, exits when pressure ≤ deadband-high
    ABORT     = 3   // press closed, vent open, LATCHED until disarm
};

// Setter used by BBController to drive a DC channel. Returns false if the
// channel is out of range. Signature matches the existing sh.channelArr helper.
using BBSetChannelFn = bool (*)(uint8_t ch1, bool state);

// Emit function for audit-trail events. Implemented in main.cpp to write to
// the primary telemetry stream. First argument is a short category code (e.g.
// "AV_ENTER"), side is 'L'/'F', detail is an optional human-readable tail.
using BBEmitFn = void (*)(const char* cat, char side, const char* detail);

class BBController {
public:
    // ptArray     — base of the V2-forwarded PT array; element [pressPtIdx]
    //               is read every update()
    // pressPtIdx  — 0-indexed PT channel for the press-line PT
    // pressDcCh   — 1-indexed DC channel for the press solenoid
    // ventDcCh    — 1-indexed DC channel for the vent solenoid; BB_DC_CH_UNSET disables vent features
    // venturiUp   — 0-indexed PT channel for upstream venturi tap; BB_PT_CH_UNSET disables mdot
    // venturiDn   — 0-indexed PT channel for downstream/throat tap; BB_PT_CH_UNSET disables mdot
    // busId       — 'L' or 'F', used for telemetry labels
    BBController(const float* ptArray,
                 uint8_t      pressPtIdx,
                 uint8_t      pressDcCh,
                 uint8_t      ventDcCh,
                 uint8_t      venturiUp,
                 uint8_t      venturiDn,
                 char         busId);

    // Wire outputs. Must be called once before update() runs.
    void bindIO(BBSetChannelFn setChannel, BBEmitFn emit);

    // Per-side config mutators. All emit EVT:CFG_PUSH on success.
    void configureCore(float setpoint, float deadband, uint32_t waitMs, uint32_t maxOpenMs);
    void configurePredictiveClose(uint32_t closeDelayMs);
    void configureVent(float triggerPsi, bool autoVentEnabled);
    void configureMdot(float mdotTarget, float spMin, float spMax,
                       float gain, float densityKgm3, bool enabled);

    // Runtime-only predictive control gate. Defaults false and is cleared by
    // forceSafe(), so EEPROM or a prior run can never silently re-enable it.
    void setPredictiveEnabled(bool enabled);

    // Enter SUSTAIN (bang-bang control). Caller must have verified armed.
    // Rejected if already non-DISABLED. Emits BB_ON / OWN_CONFLICT.
    bool enableSustain();

    // Leave SUSTAIN but stay DISABLED (press closed, vent untouched).
    // Used by GC 'b<side>0'.
    void disableSustain();

    // Manual vent: enter AUTO_VENT. Rejected if vent DC unset (AV_NO_HW).
    bool manualVent();

    // Manual close of vent: only exits AUTO_VENT (not ABORT). Returns true if
    // the state changed. For safety, to exit AUTO_VENT the pressure must be at
    // or below deadband-high; caller passes `force=true` to override (e.g. for
    // debugging before a flow).
    bool manualVentClose(bool force);

    // Latched abort. Can be triggered from any state. Cleared by disarm
    // (forceSafe) or by `b<side>0` while armed (operator acknowledge).
    bool latchAbort();

    // Called on disarm or from setup(): unconditionally close both valves,
    // clear ABORT latch, move to DISABLED. Always safe to call.
    void forceSafe();

    // Main loop tick. `armed` reflects the current master-arm state. When
    // !armed the controller self-safes and returns. `psiSettled` should be
    // false while the PT median filter is still warming up — sanity bounds are
    // not enforced until it is true.
    void update(bool armed, bool psiSettled = true, uint32_t pressureSampleMs = 0);

    // Accessors
    BBState          state()         const { return _state; }
    bool             isPressOpen()   const { return _pressOpen; }
    bool             isVentOpen()    const { return _ventOpen; }
    float            lastPressure()  const { return _lastPressure; }
    float            pressureRate()  const { return _pressureRate; }
    float            projectedPressure() const { return _projectedPressure; }
    float            predictionHorizonMs() const { return _predictionHorizonMs; }
    bool             pressureRateValid() const { return _rateValid; }
    bool             predictiveEnabled() const { return _predictiveEnabled; }
    float            lastMdot()      const { return _lastMdot; }
    const BBConfig&  config()        const { return _cfg; }
    char             busId()         const { return _busId; }
    uint8_t          pressDcCh()     const { return _pressCh; }
    uint8_t          ventDcCh()      const { return _ventCh; }
    bool             hasVentHw()     const { return _ventCh != BB_DC_CH_UNSET; }
    bool             hasVenturiHw() const { return _vUp != BB_PT_CH_UNSET && _vDn != BB_PT_CH_UNSET; }

    // For the channel-ownership check in main.cpp. Only reserves the channel
    // while BB is actively driving it (non-DISABLED) — manual control is
    // allowed whenever this side's BB is off. Exception: ABORT latched with
    // no vent hardware has nothing left to drive (press was already forced
    // closed by latchAbort()'s no-vent-HW path, and there's no vent channel
    // to hold) — release the press channel so GC has a manual way to manage
    // venting, without weakening the ABORT-only-clears-on-disarm invariant.
    bool ownsChannel(uint8_t ch1) const {
        if (_state == BBState::DISABLED) return false;
        if (_state == BBState::ABORT && !hasVentHw()) return false;
        return ch1 == _pressCh || (ch1 == _ventCh && _ventCh != BB_DC_CH_UNSET);
    }

private:
    // Hardware binding
    const float*   _ptArray;      // base pointer; abs-indexed for press + venturi
    uint8_t        _ptIdx;
    uint8_t        _pressCh;
    uint8_t        _ventCh;
    uint8_t        _vUp;
    uint8_t        _vDn;
    char           _busId;
    BBSetChannelFn _set   = nullptr;
    BBEmitFn       _emit  = nullptr;

    // Runtime
    BBConfig       _cfg;
    BBState        _state         = BBState::DISABLED;
    bool           _pressOpen     = false;
    bool           _ventOpen      = false;
    bool           _abortLatched  = false;
    bool           _predictiveEnabled = false;
    float          _lastPressure  = 0.0f;
    float          _lastMdot      = 0.0f;
    float          _pressureRate  = 0.0f;  // filtered psi/s
    float          _projectedPressure = 0.0f;
    float          _predictionHorizonMs = 0.0f;
    float          _sampleIntervalMs = 0.0f;
    float          _rateSamplePressure = 0.0f;
    uint32_t       _rateSampleMs  = 0;
    bool           _rateValid     = false;
    elapsedMillis  _switchTimer;   // debounce / slow-press wait
    elapsedMillis  _openTimer;     // tracks how long press has been open (slow-press)
    elapsedMillis  _mdotTimer;     // mdot update cadence

    // Helpers
    void _setPress(bool open, const char* reason);
    void _setVent(bool open, const char* reason);
    void _goto(BBState next, const char* reason);
    void _updateSustain();
    void _updateAutoVent();
    void _updateAbort();
    void _updateMdot();
    void _updatePressureRate(uint32_t sampleMs);
    float _computeMdot();
    void _emitSafe(const char* cat, const char* detail);
};

// EEPROM persistence. Both sides are stored in one block so their CRC is joint.
void bbLoadEeprom(BBController& lox, BBController& fuel);
void bbSaveEeprom(const BBController& lox, const BBController& fuel);
