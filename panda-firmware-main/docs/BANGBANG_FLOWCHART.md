# Bang-Bang Control — End-to-End Auditable Flowchart

This document is the source of truth for what the V1 bang-bang controller does each tick. Every branch shown here is a line of code in `src/board-functions/BangBang.cpp`; every edge emits an `EVT:` to GC.

Read these three diagrams together:
1. **State machine** — what state the controller can be in, and what moves it between states.
2. **`update()` tick** — what happens on every call to `BBController::update(armed)`.
3. **Valve driver truth table** — mapping from state to the two solenoids.

---

## 1. State machine

```mermaid
stateDiagram-v2
    [*] --> DISABLED: cold start / forceSafe()

    DISABLED --> SUSTAIN: GC 'b<side>1'\n(requires gArmed)\nemit BB_ON
    SUSTAIN --> DISABLED: GC 'b<side>0'\nemit BB_OFF

    SUSTAIN --> AUTO_VENT: pressure > autovent_trigger\n(only if autovent_enabled && hasVentHw)\nemit AV_ENTER
    SUSTAIN --> AUTO_VENT: GC 'v<side>1'\n(requires gArmed, hasVentHw)\nemit AV_ENTER
    DISABLED --> AUTO_VENT: GC 'v<side>1'\nemit AV_ENTER

    AUTO_VENT --> DISABLED: pressure <= deadband_high\nemit AV_EXIT
    AUTO_VENT --> DISABLED: GC 'v<side>0' && pressure ok\nemit AV_EXIT

    SUSTAIN   --> ABORT: GC 'x<side>'\nemit ABORT_ENTER
    AUTO_VENT --> ABORT: GC 'x<side>'\nemit ABORT_ENTER
    DISABLED  --> ABORT: GC 'x<side>'\nemit ABORT_ENTER
    SUSTAIN   --> ABORT: SANITY_FAIL (PT out of bounds)\nemit SANITY_FAIL + ABORT_ENTER
    AUTO_VENT --> ABORT: SANITY_FAIL\nemit SANITY_FAIL + ABORT_ENTER

    ABORT --> DISABLED: GC 'r' (disarm) or 'b<side>0' (ack)\nemit ABORT_CLEAR + BB_OFF

    note right of ABORT
      Latched. There is no
      "unabort" command by design.
      forceSafe() on disarm is
      the only clearing path.
    end note

    note left of SUSTAIN
      Mass-flow correction runs
      here IFF mdot_enabled &&
      hasVenturiHw. Nudges setpoint
      within [sp_min, sp_max]
      every BB_MDOT_UPDATE_MS.
    end note
```

---

## 2. `update(armed)` per-tick flow

```mermaid
flowchart TD
    A[update armed] --> B{armed?}
    B -- no --> C{any non-safe state<br/>or valves open<br/>or abort latched?}
    C -- yes --> D[forceSafe:<br/>press=0, vent=0,<br/>latch=0, state=DISABLED]
    C -- no --> Z1[return]
    D --> Z1

    B -- yes --> E[lastPressure = ptArray&lbrack;ptIdx&rbrack;]
    E --> F{state == DISABLED?}
    F -- yes --> M[mass-flow update<br/>skipped, state != SUSTAIN]
    F -- no --> G{PT NaN or<br/>outside MIN..MAX?}
    G -- yes --> H[emit SANITY_FAIL<br/>latchAbort]
    H --> Z2[return]
    G -- no --> I{state}

    I -- SUSTAIN --> J1{autovent_enabled &&<br/>hasVentHw &&<br/>PT > autovent_trigger?}
    J1 -- yes --> J2[press=0, vent=1<br/>state=AUTO_VENT<br/>emit AV_ENTER]
    J1 -- no --> K1{press open &&<br/>max_open_ms > 0 &&<br/>openTimer >= max_open_ms?}
    K1 -- yes --> K2[press=0<br/>switchTimer=0<br/>slow-press cap]
    K1 -- no --> L1{switchTimer < wait_ms?}
    L1 -- yes --> M
    L1 -- no --> L2{PT > hi && press open?}
    L2 -- yes --> L3[press=0<br/>switchTimer=0]
    L2 -- no --> L4{PT < lo && press closed?}
    L4 -- yes --> L5[press=1<br/>switchTimer=0]
    L4 -- no --> M

    I -- AUTO_VENT --> N1{PT <= deadband_high?}
    N1 -- yes --> N2[vent=0<br/>state=DISABLED<br/>emit AV_EXIT]
    N1 -- no --> M

    I -- ABORT --> O1[hold: no action]
    O1 --> M

    J2 --> M
    K2 --> M
    L3 --> M
    L5 --> M
    N2 --> M

    M --> M1{mdot_enabled &&<br/>hasVenturiHw &&<br/>density_kgm3 > 0 &&<br/>state == SUSTAIN &&<br/>mdotTimer >= 500ms?}
    M1 -- no --> Z3[done]
    M1 -- yes --> M2[dP = max 0, Pup-Pdn then psi→Pa<br/>mdot = CdA · √ 2·ρ·dP<br/>err = target - mdot<br/>newSp = sp + gain·err<br/>clamp to sp_min..sp_max]
    M2 --> M3{newSp != sp?}
    M3 -- yes --> M4[sp = newSp<br/>emit MDOT_ADJ<br/>or MDOT_CLAMP]
    M3 -- no --> Z3
    M4 --> Z3
```

---

## 3. Valve truth table

| State | Press solenoid | Vent solenoid | Notes |
|---|---|---|---|
| `DISABLED` | closed | closed | Safe default. Manual `S` commands permitted on non-BB channels. |
| `SUSTAIN` | toggled by bang-bang within `[sp − db/2, sp + db/2]` | closed | If `max_open_ms > 0`, press is force-closed after that many ms continuously open and held closed until `wait_ms` elapses. |
| `AUTO_VENT` | closed | open | Holds until `PT ≤ sp + db/2`, then drops to `DISABLED`. |
| `ABORT` | closed | open (if `hasVentHw`) | Latched. Only disarm clears. Without vent HW, press is still closed and `AV_NO_HW` is emitted. |

**Disarm (`r`) always wins.** It calls `forceSafe()` which unconditionally closes both valves, clears the ABORT latch, and moves to `DISABLED`, regardless of prior state.

---

## 4. Command → handler → state map

| GC command | Handler | Resulting call | Allowed from |
|---|---|---|---|
| `B<side><sp>,<db>,<wait>,<maxOpen>` | `handleB` | `configureCore()` + `bbSaveEeprom()` | Any state |
| `V<side><trig>,<autoOn>` | `handleV` | `configureVent()` + `bbSaveEeprom()` | Any state |
| `M<side><mdot>,<spMin>,<spMax>,<gain>,<rho>,<on>` | `handleM` | `configureMdot()` + `bbSaveEeprom()` | Any state |
| `b<side>1` | `handleLowerB` | `enableSustain()` | `DISABLED` and `gArmed` |
| `b<side>0` | `handleLowerB` | `disableSustain()` | Any except `ABORT` |
| `v<side>1` | `handleLowerV` | `manualVent()` | Any except `ABORT`, requires `gArmed` + `hasVentHw` |
| `v<side>0` | `handleLowerV` | `manualVentClose(force=false)` | Only `AUTO_VENT`, requires pressure ≤ deadband-high |
| `x<side>` | `handleLowerX` | `latchAbort()` | Any state |
| `a` | inline | sets `gArmed = true` | Any state |
| `r` | inline | `forceSafe()` both sides | Any state |

---

## 5. Event emission — where each EVT is triggered

| `EVT` category | Emitted from | Condition |
|---|---|---|
| `CFG_PUSH` | `configureCore/Vent/Mdot` | Any successful config write (also during EEPROM load) |
| `BB_ON` | `_goto(SUSTAIN, …)` | `enableSustain()` success |
| `BB_OFF` | `_goto(DISABLED, …)` | `disableSustain()`, `AV_EXIT`, `forceSafe()` |
| `VALVE` | `_setPress`, `_setVent` | Every edge on either solenoid, with reason string |
| `AV_ENTER` | `_goto(AUTO_VENT, …)` | Manual `v…1` or auto-trigger |
| `AV_EXIT` | `manualVentClose`, `_updateAutoVent` | On close path; paired with `BB_OFF` |
| `AV_REJECT_CLOSE` | `manualVentClose` | Refused because pressure too high |
| `AV_NO_HW` | `manualVent`, `latchAbort` | Vent DC unset |
| `ABORT_ENTER` | `_goto(ABORT, …)` | `latchAbort()` or `SANITY_FAIL` |
| `ABORT_CLEAR` | `forceSafe` | Disarm clears a latched abort |
| `SANITY_FAIL` | `update` | PT NaN or outside `BB_PRESSURE_*_PSI` while non-`DISABLED` |
| `OWN_CONFLICT` | `enableSustain`, `disableSustain`, `manualVent` | Command rejected by state preconditions |
| `MDOT_ADJ` / `MDOT_CLAMP` | `_updateMdot` | Setpoint nudged / saturated at bound |
| `COMMS_WD_ARM` / `COMMS_LOSS` / `COMMS_OK` / `COMMS_DISARM` | `serviceGcLinkWatchdog` (main.cpp) | Primary GC link armed / timed out / restored / silent past the disarm threshold. Side field is `-`, not `L`/`F` — these are link-wide, not per-side. |

Every row of this table is a line of code. If you add a new state edge, you add a new row here.

---

## 6. What's intentionally out of scope (phase 2)

- **Venturi calibration.** `_computeMdot()` uses the incompressible Bernoulli form `m_dot = CdA · √(2·ρ·ΔP)`, with CdA = `BB_VENTURI_CDA_M2` (single board constant, currently 3.22e-5 m²) and per-side `density_kgm3` supplied via the `M` command. ΔP is `P_up − P_dn` clamped ≥ 0, converted psi → Pa. Throat/upstream areas are absorbed into CdA; update the constant if the venturi geometry changes.
- **Primary-link (GC) watchdog.** Any line from GC refreshes it; GC must send `h` at 5 Hz so a quiet hold is distinguishable from a severed cable. After 600 ms both controllers force-safe (`ABORT` exempt — its vent stays open); after 10 s the board disarms itself. Dormant until the first `h` of the boot, so it cannot nuisance-disarm against a GC that does not beat — watch `LINK:<armed>`. See §8 of `GC_USERS_GUIDE.md`.
- **Secondary-link staleness detector.** A complete numeric 2-channel V2 frame refreshes the watchdog. If it expires after 50 ms while BB is active, that controller force-safes, emits `PT_STALE`, and requires explicit operator re-enable.
- **AUTO_VENT → SUSTAIN auto-recovery.** Current behavior drops to `DISABLED` on `AV_EXIT` by design — requires explicit operator re-enable. Change only after an explicit ops decision.
