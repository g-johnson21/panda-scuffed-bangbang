#include "board-functions/BangBang.hpp"
#include <EEPROM.h>
#include <math.h>

// =============================================================================
// BBController implementation.
//
// State model:
//
//            ┌─────────────┐  enableSustain()         ┌──────────────┐
//            │  DISABLED   │ ───────────────────────▶ │   SUSTAIN    │
//            │ (press off, │                          │ (bang-bang)  │
//            │  vent off)  │ ◀────────── disableSustain/forceSafe ── │
//            └─────────────┘                          └──────┬───────┘
//               ▲    ▲                                       │
//               │    │  manualVentClose(pressure ok)         │ autovent_trigger hit
//               │    │                                       │ OR manualVent()
//               │    │                                       ▼
//       forceSafe(disarm)                            ┌──────────────┐
//               │                                    │  AUTO_VENT   │
//               │                                    │ (press off,  │
//               │                                    │  vent open)  │
//               │                                    └──────┬───────┘
//               │                                           │ latchAbort()
//               │                                           ▼
//               │                                    ┌──────────────┐
//               └──────────── forceSafe (disarm) ─── │    ABORT     │
//                                                    │  (latched)   │
//                                                    └──────────────┘
//
// Every transition emits an EVT: line. forceSafe() is the only path that
// clears an ABORT latch — there is no "unabort" GC command by design.
// =============================================================================

BBController::BBController(const float* ptArray,
                           uint8_t      pressPtIdx,
                           uint8_t      pressDcCh,
                           uint8_t      ventDcCh,
                           uint8_t      venturiUp,
                           uint8_t      venturiDn,
                           char         busId)
    : _ptArray(ptArray),
      _ptIdx(pressPtIdx),
      _pressCh(pressDcCh),
      _ventCh(ventDcCh),
      _vUp(venturiUp),
      _vDn(venturiDn),
      _busId(busId) {}

void BBController::bindIO(BBSetChannelFn setChannel, BBEmitFn emit) {
    _set  = setChannel;
    _emit = emit;
}

void BBController::_emitSafe(const char* cat, const char* detail) {
    if (!_emit) return;
    char buf[96];
    const float pt = _ptArray[_ptIdx];
    if (!detail || detail[0] == '\0')
        snprintf(buf, sizeof(buf), "pt=%.1f", pt);
    else if (strncmp(detail, "pt=", 3) == 0)
        snprintf(buf, sizeof(buf), "%s", detail);
    else
        snprintf(buf, sizeof(buf), "%s,pt=%.1f", detail, pt);
    _emit(cat, _busId, buf);
}

// ── Config ────────────────────────────────────────────────────────────────

void BBController::configureCore(float setpoint, float deadband, uint32_t waitMs, uint32_t maxOpenMs) {
    _cfg.setpoint_psi = setpoint;
    _cfg.deadband_psi = deadband;
    _cfg.wait_ms      = waitMs;
    _cfg.max_open_ms  = maxOpenMs;
    char buf[64];
    snprintf(buf, sizeof(buf), "sp=%.1f,db=%.1f,wait=%lu,maxOpen=%lu",
             setpoint, deadband, (unsigned long)waitMs, (unsigned long)maxOpenMs);
    _emitSafe("CFG_PUSH", buf);
}

void BBController::configureVent(float triggerPsi, bool autoVentEnabled) {
    _cfg.autovent_trigger = triggerPsi;
    _cfg.autovent_enabled = autoVentEnabled;
    char buf[48];
    snprintf(buf, sizeof(buf), "avTrig=%.1f,avAuto=%d", triggerPsi, autoVentEnabled ? 1 : 0);
    _emitSafe("CFG_PUSH", buf);
}

void BBController::configureMdot(float mdotTarget, float spMin, float spMax,
                                 float gain, float densityKgm3, bool enabled) {
    _cfg.mdot_target  = mdotTarget;
    _cfg.sp_min       = spMin;
    _cfg.sp_max       = spMax;
    _cfg.mdot_gain    = gain;
    _cfg.density_kgm3 = densityKgm3;
    _cfg.mdot_enabled = enabled;
    char buf[112];
    snprintf(buf, sizeof(buf), "mdot=%.3f,spMin=%.1f,spMax=%.1f,gain=%.4f,rho=%.1f,on=%d",
             mdotTarget, spMin, spMax, gain, densityKgm3, enabled ? 1 : 0);
    _emitSafe("CFG_PUSH", buf);
}

// ── State transitions ─────────────────────────────────────────────────────

void BBController::_setPress(bool open, const char* reason) {
    if (open == _pressOpen) return;
    if (_set) _set(_pressCh, open);
    _pressOpen = open;
    if (open) _openTimer = 0;
    char buf[48];
    snprintf(buf, sizeof(buf), "press=%d,reason=%s", open ? 1 : 0, reason ? reason : "");
    _emitSafe("VALVE", buf);
}

void BBController::_setVent(bool open, const char* reason) {
    if (!hasVentHw()) return;
    if (open == _ventOpen) return;
    if (_set) _set(_ventCh, open);
    _ventOpen = open;
    char buf[48];
    snprintf(buf, sizeof(buf), "vent=%d,reason=%s", open ? 1 : 0, reason ? reason : "");
    _emitSafe("VALVE", buf);
}

void BBController::_goto(BBState next, const char* reason) {
    if (next == _state) return;
    const char* evt = "STATE";
    switch (next) {
        case BBState::DISABLED:  evt = "BB_OFF";       break;
        case BBState::SUSTAIN:   evt = "BB_ON";        break;
        case BBState::AUTO_VENT: evt = "AV_ENTER";     break;
        case BBState::ABORT:     evt = "ABORT_ENTER";  break;
    }
    _state = next;
    _switchTimer = 0;
    _emitSafe(evt, reason);
}

bool BBController::enableSustain() {
    if (_state != BBState::DISABLED) {
        _emitSafe("OWN_CONFLICT", "enable while non-disabled");
        return false;
    }
    _goto(BBState::SUSTAIN, "GC enable");
    return true;
}

void BBController::disableSustain() {
    if (_state == BBState::ABORT) {
        _setVent(false, "abort clear");
        _abortLatched = false;
        _goto(BBState::DISABLED, "abort clear (b0)");
        _emitSafe("ABORT_CLEAR", "operator b0");
        return;
    }
    _setPress(false, "disable");
    // Do not touch vent — if we were in AUTO_VENT, caller must manualVentClose.
    if (_state == BBState::SUSTAIN) {
        _goto(BBState::DISABLED, "GC disable");
    }
}

bool BBController::manualVent() {
    if (!hasVentHw()) {
        _emitSafe("AV_NO_HW", "vent DC channel unset");
        return false;
    }
    if (_state == BBState::ABORT) {
        _emitSafe("OWN_CONFLICT", "manualVent ignored while ABORT latched");
        return false;
    }
    _setPress(false, "manualVent");
    _setVent(true, "manualVent");
    _goto(BBState::AUTO_VENT, "manualVent");
    return true;
}

bool BBController::manualVentClose(bool force) {
    if (_state != BBState::AUTO_VENT) return false;
    if (!force) {
        const float hi = _cfg.setpoint_psi + _cfg.deadband_psi * 0.5f;
        if (_lastPressure > hi) {
            _emitSafe("AV_REJECT_CLOSE", "pressure still above deadband-high");
            return false;
        }
    }
    _setVent(false, "manualVentClose");
    _goto(BBState::DISABLED, force ? "manualVentClose(force)" : "manualVentClose");
    _emitSafe("AV_EXIT", force ? "forced" : "pressure ok");
    return true;
}

bool BBController::latchAbort() {
    if (!hasVentHw()) {
        _emitSafe("AV_NO_HW", "abort requested but vent unset");
        // Still close press so we have at least *some* safing.
        _setPress(false, "abort-no-hw");
        _abortLatched = true;
        _goto(BBState::ABORT, "abort (no vent HW)");
        return false;
    }
    _setPress(false, "abort");
    _setVent(true, "abort");
    _abortLatched = true;
    _goto(BBState::ABORT, "latchAbort");
    return true;
}

void BBController::forceSafe() {
    _setPress(false, "forceSafe");
    _setVent(false, "forceSafe");
    bool wasLatched = _abortLatched;
    _abortLatched = false;
    if (_state != BBState::DISABLED) {
        _goto(BBState::DISABLED, wasLatched ? "disarm (ABORT cleared)" : "forceSafe");
    }
    if (wasLatched) _emitSafe("ABORT_CLEAR", "disarm");
}

// ── Main loop tick ────────────────────────────────────────────────────────

void BBController::update(bool armed, bool psiSettled) {
    // Always snapshot live PT for the 1 Hz BB heartbeat GC reads, even when
    // disarmed or DISABLED — lastPressure must not stick at 0 across disarm.
    _lastPressure = _ptArray[_ptIdx];

    if (!armed) {
        if (_state != BBState::DISABLED || _pressOpen || _ventOpen || _abortLatched) {
            forceSafe();
        }
        return;
    }

    // Sanity bounds apply in every non-DISABLED, non-ABORT state. ABORT is
    // already latched safe (valves parked) and only forceSafe()/disarm clears
    // it — re-checking here would just re-emit SANITY_FAIL/latchAbort every
    // tick forever, flooding the telemetry stream and starving Serial2 TX.
    // Also defer until the median filter has filled — early samples are not
    // representative and must not trip SANITY_FAIL on enable.
    if (psiSettled &&
        _state != BBState::DISABLED && _state != BBState::ABORT) {
        if (isnanf(_lastPressure) ||
            _lastPressure < BB_PRESSURE_MIN_PSI ||
            _lastPressure > BB_PRESSURE_MAX_PSI) {
            _emitSafe("SANITY_FAIL", "out of bounds");
            latchAbort();
            return;
        }
    }

    switch (_state) {
        case BBState::DISABLED:                       break;
        case BBState::SUSTAIN:    _updateSustain();   break;
        case BBState::AUTO_VENT:  _updateAutoVent();  break;
        case BBState::ABORT:      _updateAbort();     break;
    }

    _updateMdot();
}

void BBController::_updateSustain() {
    // Auto-vent trigger takes precedence over bang-bang.
    if (_cfg.autovent_enabled &&
        hasVentHw() &&
        _lastPressure > _cfg.autovent_trigger) {
        _setPress(false, "autovent trigger");
        _setVent(true,  "autovent trigger");
        _goto(BBState::AUTO_VENT, "autoTrigger");
        return;
    }

    const float hi = _cfg.setpoint_psi + _cfg.deadband_psi * 0.5f;
    const float lo = _cfg.setpoint_psi - _cfg.deadband_psi * 0.5f;

    // Slow-press: if max_open_ms > 0 and press has been open for that long,
    // force a close and enforce wait_ms before another open is allowed.
    if (_pressOpen && _cfg.max_open_ms > 0 && _openTimer >= _cfg.max_open_ms) {
        _setPress(false, "slowPress cap");
        _switchTimer = 0;
        return;
    }

    if (_switchTimer < _cfg.wait_ms) return;

    if (_lastPressure > hi && _pressOpen) {
        _setPress(false, "above hi");
        _switchTimer = 0;
    } else if (_lastPressure < lo && !_pressOpen) {
        _setPress(true, "below lo");
        _switchTimer = 0;
    }
}

void BBController::_updateAutoVent() {
    // Exit condition: pressure has come back to or below deadband-high.
    const float hi = _cfg.setpoint_psi + _cfg.deadband_psi * 0.5f;
    if (_lastPressure <= hi) {
        _setVent(false, "autovent exit");
        _emitSafe("AV_EXIT", "pressure below hi");
        // Return to SUSTAIN if autovent was auto-triggered and operator wants
        // continued control. Conservative default: drop to DISABLED so the
        // operator must explicitly re-arm BB. Safer, requires an explicit
        // decision before we start driving valves again.
        _goto(BBState::DISABLED, "AV_EXIT→DISABLED");
    }
}

void BBController::_updateAbort() {
    // Latched — nothing to do. Valves were set at latchAbort(). Only
    // forceSafe() (triggered by disarm) clears this state.
}

// ── Mass-flow correction ──────────────────────────────────────────────────

float BBController::_computeMdot() {
    // Incompressible Bernoulli venturi:
    //   m_dot [kg/s] = CdA · sqrt( 2 · ρ · ΔP )
    // CdA is a single board constant (BB_VENTURI_CDA_M2). Density is per-side
    // so LOX vs Fuel can be distinguished. Requires both venturi PTs wired
    // and density > 0 — otherwise returns NaN and the caller skips adjustment.
    if (!hasVenturiHw()) return NAN;
    if (_cfg.density_kgm3 <= 0.0f) return NAN;
    float pUp  = _ptArray[_vUp];
    float pDn  = _ptArray[_vDn];
    float dPsi = pUp - pDn;
    if (dPsi < 0.0f) dPsi = 0.0f;
    float dPa  = dPsi * PSI_TO_PA;
    return BB_VENTURI_CDA_M2 * sqrtf(2.0f * _cfg.density_kgm3 * dPa);
}

void BBController::_updateMdot() {
    if (!_cfg.mdot_enabled) return;
    if (!hasVenturiHw()) return;
    if (_state != BBState::SUSTAIN) return;   // only adjust during active control
    if (_mdotTimer < BB_MDOT_UPDATE_MS) return;
    _mdotTimer = 0;

    float mdot = _computeMdot();
    if (isnanf(mdot)) return;
    _lastMdot = mdot;

    float error = _cfg.mdot_target - mdot;
    float delta = _cfg.mdot_gain * error;
    float newSp = _cfg.setpoint_psi + delta;

    bool clamped = false;
    if (newSp < _cfg.sp_min) { newSp = _cfg.sp_min; clamped = true; }
    if (newSp > _cfg.sp_max) { newSp = _cfg.sp_max; clamped = true; }

    if (newSp != _cfg.setpoint_psi) {
        _cfg.setpoint_psi = newSp;
        char buf[64];
        snprintf(buf, sizeof(buf), "mdot=%.3f,err=%.3f,sp=%.1f", mdot, error, newSp);
        _emitSafe(clamped ? "MDOT_CLAMP" : "MDOT_ADJ", buf);
    }
}

// ── EEPROM persistence ────────────────────────────────────────────────────

static uint8_t computeCrc(const BBConfig& lox, const BBConfig& fuel) {
    uint8_t crc = 0;
    const uint8_t* p;
    p = reinterpret_cast<const uint8_t*>(&lox);
    for (size_t i = 0; i < sizeof(BBConfig); i++) crc ^= p[i];
    p = reinterpret_cast<const uint8_t*>(&fuel);
    for (size_t i = 0; i < sizeof(BBConfig); i++) crc ^= p[i];
    return crc;
}

void bbLoadEeprom(BBController& lox, BBController& fuel) {
    BBEepromBlock block;
    EEPROM.get(BB_EEPROM_ADDR, block);

    if (block.magic != BB_EEPROM_MAGIC) return;
    if (block.crc != computeCrc(block.lox, block.fuel)) return;

    lox.configureCore(block.lox.setpoint_psi, block.lox.deadband_psi,
                      block.lox.wait_ms, block.lox.max_open_ms);
    lox.configureVent(block.lox.autovent_trigger, block.lox.autovent_enabled);
    lox.configureMdot(block.lox.mdot_target, block.lox.sp_min, block.lox.sp_max,
                      block.lox.mdot_gain, block.lox.density_kgm3, block.lox.mdot_enabled);

    fuel.configureCore(block.fuel.setpoint_psi, block.fuel.deadband_psi,
                       block.fuel.wait_ms, block.fuel.max_open_ms);
    fuel.configureVent(block.fuel.autovent_trigger, block.fuel.autovent_enabled);
    fuel.configureMdot(block.fuel.mdot_target, block.fuel.sp_min, block.fuel.sp_max,
                       block.fuel.mdot_gain, block.fuel.density_kgm3, block.fuel.mdot_enabled);
}

void bbSaveEeprom(const BBController& lox, const BBController& fuel) {
    BBEepromBlock block;
    block.magic = BB_EEPROM_MAGIC;
    block.lox   = lox.config();
    block.fuel  = fuel.config();
    block.crc   = computeCrc(block.lox, block.fuel);
    EEPROM.put(BB_EEPROM_ADDR, block);
}
