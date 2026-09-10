#include <EEPROM.h>
#include <FlexSerial.h>
#include <SPI.h>
#include <Wire.h>

#include "hardware-configs/pins.hpp"

// #include "scanners/Scanner.hpp"
#include "scanners/FScanner.hpp" // Fluids DAQ
#include "scanners/SScanner.hpp" // Solenoid current DAQ

#include "board-functions/ArmingController.hpp"
#include "board-functions/BangBang.hpp"
#include "dc-controllers/SequenceHandler.hpp"
#include "telemetry/TelemetryHandler.hpp"

#include "drivers/MCP9802A0.hpp" // Temperature sensors

/**
 * SERIAL PROTOCOL (Baud rate: 460800)
 *
 * See docs/GC_USERS_GUIDE.md for the canonical command + telemetry reference.
 * Summary:
 *
 * Commands (Serial2 ← GC):
 *   'a'                                 Arm
 *   'r'                                 Disarm (also forceSafe()s BB)
 *   'S<chHex><state>'                   Direct solenoid command
 *   's<chHex><state>.<ms 5-digit>'      Sequence step append
 *   'f'                                 Fire loaded sequence
 *   'B<side><sp>,<db>,<wait>,<maxOpen>' Configure BB core (L or F)
 *   'D<side><closeMs>'                  Configure predictive close delay
 *   'V<side><trig>,<autoOn01>'          Configure BB auto-vent
 *   'M<side><mdot>,<spMin>,<spMax>,<gain>,<rho>,<on01>'  Configure BB massflow
 *   'b<side><0|1>'                      Arm/disarm bang-bang control
 *   'e<side><0|1>'                      Enable/disable predictive cutoff
 *   'v<side><0|1>'                      Manual vent open(1) / close(0)
 *   'x<side>'                           Latched abort (cleared only by 'r')
 *   'h'                                 GC heartbeat (see link watchdog below)
 *   'TL' / 'TF'                         Tare LOX/Fuel PT to current PSI (persisted)
 *   'Tz'                                Clear all PT tare offsets
 *   'T<n>,<offset>'                     Set explicit PSI offset for channel n
 *
 * Telemetry (Serial2 → GC):
 *   'p<f0>,p<f1>'      — 2 PT signal values forwarded from V2
 *   'P<f0>,P<f1>'      — 2 PT pressures (PSI), scaled on V1 from p-values
 *   's<f0>,s<f1>,...'   — 12 solenoid current voltages (V1 local)
 *   't<f0>,t<f1>,...'   — 12 LC+TC voltages (V1 local)
 *   'BB:<L|F>:<state>:<press>:<vent>:<pressure>'   1 Hz summary heartbeat
 *   'BBD:<side>:<state>:<press>:<pressure>:<rate>:<projected>:<hi>:<delay>:<horizon>:<valid>:<enabled>'
 *                                                   10 Hz predictive debug
 *   'LINK:<armed>:<lost>:<silentMs>'                1 Hz link-watchdog status
 *   'EVT:<ms>:<cat>:<L|F>:<detail>'                audit event on every BB
 * transition
 *
 * GC LINK WATCHDOG
 * GC must send 'h' at 5 Hz. Any line from GC counts as liveness; 'h' exists so
 * GC can prove liveness without commanding anything. After COMMS_LOSS_MS of
 * silence both bang-bang controllers are forced safe; after COMMS_DISARM_MS
 * the board disarms itself exactly as if 'r' had been received. The watchdog
 * is dormant until the first 'h' of the boot, so firmware built ahead of a
 * heartbeat-capable GC cannot nuisance-disarm — watch LINK:<armed> to confirm
 * it is actually active. Recovery is never automatic: re-arm and re-enable.
 *
 * Serial5 is a direct TTL crossover from Panda V2 carrying PT CSV rows only.
 * The RS-485 transceiver on this bus is bypassed: V1 pin 20 → V2 pin 25,
 * V2 pin 24 → V1 pin 21, common GND. No DE control. Short-run point-to-point.
 */

TelemetryHandler th(Serial2, 1024);      // primary ← GC
TelemetryHandler thXover(Serial5, 1024); // secondary ← V2 PT crossover
SequenceHandler sh;
ArmingController ac(PIN_ARM, PIN_DISARM);

SScanner sScanner(sADCPins.cs, sADCPins.irq, SPI, SPISettingsDefault);
FScanner fScanner(ptADCPins.cs, ptADCPins.irq, SPI1, SPISettingsDefault);

// ── V2-forwarded PT data (filled by secondary-bus parser) ─────────────────
static float v2PtData[NUM_PT_CHANNELS] = {0};
// Scaled PT view (PSI) derived from v2PtData[] for GC consumption.
static float v2PtPsiData[NUM_PT_CHANNELS] = {0};
static uint32_t lastV2PtMs = 0;
static bool hasValidV2Pt = false;

// Mirror of master-arm state (what BB gates on).
static bool gArmed = false;

// ── GC primary-link health (Serial2) ─────────────────────────────────────
// lastGcRxMs is refreshed by ANY complete line from GC, not just heartbeats:
// a command is proof of life too. gcWatchdogArmed latches on the first 'h' of
// the boot and never clears, so a GC that stops heartbeating after having
// started cannot silently switch the protection back off.
static uint32_t lastGcRxMs = 0;
static bool gcWatchdogArmed = false;
static bool gcLinkLost = false;   // stage 1 tripped, not yet recovered
static bool gcDisarmDone = false; // stage 2 fired for this outage

// ── Bang-bang controllers ────────────────────────────────────────────────
// BB configuration and safety limits are in PSI, so controllers must consume
// the scaled view rather than the raw 4-20 mA telemetry values.
BBController bbLox(v2PtPsiData, BB_LOX_PT_CH, BB_LOX_DC_CH, BB_LOX_VENT_DC_CH,
                   BB_LOX_VENTURI_UP_PT, BB_LOX_VENTURI_DN_PT, 'L');
BBController bbFuel(v2PtPsiData, BB_FUEL_PT_CH, BB_FUEL_DC_CH, BB_FUEL_VENT_DC_CH,
                    BB_FUEL_VENTURI_UP_PT, BB_FUEL_VENTURI_DN_PT, 'F');

// ── BB → DC channel setter ───────────────────────────────────────────────
static bool bbSetChannel(uint8_t ch1, bool state) {
  if (ch1 < 1 || ch1 > NUM_DC_CHANNELS)
    return false;
  sh.channelArr[ch1 - 1].setState(state);
  return true;
}

// ── BB → audit-event emitter ─────────────────────────────────────────────
// Format: EVT:<ms>:<cat>:<side>:<detail>\n
static void bbEmit(const char *cat, char side, const char *detail) {
  Serial2.print("EVT:");
  Serial2.print(millis());
  Serial2.print(':');
  Serial2.print(cat);
  Serial2.print(':');
  Serial2.print(side);
  Serial2.print(':');
  Serial2.println(detail ? detail : "");
}

// ── Channel-ownership check (manual S commands blocked on BB channels) ───
static bool isBbOwned(uint8_t ch1) {
  return bbLox.ownsChannel(ch1) || bbFuel.ownsChannel(ch1);
}

static float ptTarePsiOffset[NUM_PT_CHANNELS] = {0};

struct PtTareEepromBlock {
  uint16_t magic;
  float offset_psi[NUM_PT_CHANNELS];
  uint8_t crc;
};

static uint8_t ptTareComputeCrc(const PtTareEepromBlock &block) {
  uint8_t crc = 0;
  const uint8_t *p = reinterpret_cast<const uint8_t *>(block.offset_psi);
  for (size_t i = 0; i < sizeof(block.offset_psi); i++)
    crc ^= p[i];
  return crc;
}

static void ptTareLoadEeprom() {
  PtTareEepromBlock block;
  EEPROM.get(PT_TARE_EEPROM_ADDR, block);
  if (block.magic != PT_TARE_EEPROM_MAGIC)
    return;
  if (block.crc != ptTareComputeCrc(block))
    return;
  memcpy(ptTarePsiOffset, block.offset_psi, sizeof(ptTarePsiOffset));
}

static void ptTareSaveEeprom() {
  PtTareEepromBlock block;
  block.magic = PT_TARE_EEPROM_MAGIC;
  memcpy(block.offset_psi, ptTarePsiOffset, sizeof(block.offset_psi));
  block.crc = ptTareComputeCrc(block);
  EEPROM.put(PT_TARE_EEPROM_ADDR, block);
}

static float ptSignalToPsi(float signalVolts) {
  const float currentMa = (signalVolts / PT_SHUNT_OHMS) * 1000.0f;
  return (currentMa - PT_ZERO_MA) * (PT_FULL_SCALE_PSI / PT_SPAN_MA);
}

static float ptSignalToTaredPsi(float signalVolts, uint8_t ch) {
  return ptSignalToPsi(signalVolts) - ptTarePsiOffset[ch];
}

static bool hasFreshV2Pt() {
  return hasValidV2Pt && (millis() - lastV2PtMs <= BB_PT_STALE_MS);
}

// ── PSI median filter ────────────────────────────────────────────────────
// Mitigation for sustained switching noise (e.g. arm relay/contactor
// coupling into the PT analog front-end) riding on top of an otherwise
// real signal. A rolling median over PT_PSI_MEDIAN_WINDOW samples (~250 ms
// at V2's ~290 Hz cadence) rejects that noise without hiding a genuine,
// continuously-out-of-range fault — BB's sanity check still sees the
// filtered value, so a real fault still trips it, just without chasing
// every single noisy sample. Raw v2PtData is left unfiltered so GC retains
// the true signal for diagnostics.
struct PsiMedianFilter {
  float ring[PT_PSI_MEDIAN_WINDOW];
  uint8_t count = 0;
  uint8_t idx = 0;
};

static PsiMedianFilter ptPsiFilters[NUM_PT_CHANNELS];

static void resetMedianFilter(uint8_t ch) { ptPsiFilters[ch] = PsiMedianFilter{}; }

static bool ptPsiSettled() {
  if (!hasFreshV2Pt())
    return false;
  for (uint8_t ch = 0; ch < NUM_PT_CHANNELS; ch++) {
    if (ptPsiFilters[ch].count < PT_PSI_MEDIAN_WINDOW)
      return false;
  }
  return true;
}

static float medianFilterPsi(uint8_t ch, float sample) {
  PsiMedianFilter &f = ptPsiFilters[ch];

  f.ring[f.idx] = sample;
  f.idx = (f.idx + 1) % PT_PSI_MEDIAN_WINDOW;
  if (f.count < PT_PSI_MEDIAN_WINDOW)
    f.count++;

  float sorted[PT_PSI_MEDIAN_WINDOW];
  const uint8_t n = f.count;
  for (uint8_t i = 0; i < n; i++) {
    const uint8_t ri =
        (f.idx + PT_PSI_MEDIAN_WINDOW - n + i) % PT_PSI_MEDIAN_WINDOW;
    sorted[i] = f.ring[ri];
  }

  for (uint8_t i = 1; i < n; i++) {
    const float key = sorted[i];
    int8_t j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }
  return sorted[n / 2];
}

// ── Parse PT CSV from V2 into v2PtData[] ────────────────────────────────
// Parse into temporaries and commit only a complete, numeric frame.
// An idle-timeout fragment must not refresh the BB safety watchdog or mix new
// leading channels with stale trailing channels.
static bool parseV2PtPacket(char *packet) {
  if (!packet || packet[0] == '\0')
    return false;

  float raw[NUM_PT_CHANNELS];
  float psi[NUM_PT_CHANNELS];
  uint8_t idx = 0;
  char *tok = strtok(packet, ",");
  while (tok && idx < NUM_PT_CHANNELS) {
    const char *num = (tok[0] == PT_IDENTIFIER) ? tok + 1 : tok;
    char *end = nullptr;
    const float signalVolts = strtof(num, &end);
    if (end == num || *end != '\0' || !isfinite(signalVolts))
      return false;
    raw[idx] = signalVolts;
    psi[idx] = ptSignalToTaredPsi(signalVolts, idx);
    idx++;
    tok = strtok(nullptr, ",");
  }

  if (idx != NUM_PT_CHANNELS || tok != nullptr)
    return false;

  for (uint8_t ch = 0; ch < NUM_PT_CHANNELS; ch++)
    psi[ch] = medianFilterPsi(ch, psi[ch]);

  memcpy(v2PtData, raw, sizeof(v2PtData));
  memcpy(v2PtPsiData, psi, sizeof(v2PtPsiData));
  lastV2PtMs = millis();
  hasValidV2Pt = true;
  return true;
}

// ── BB command dispatch ──────────────────────────────────────────────────
// Each handler returns true if the packet was consumed as a BB command so
// main can skip further parsing.
static BBController *pickSide(char c) {
  if (c == 'L')
    return &bbLox;
  if (c == 'F')
    return &bbFuel;
  return nullptr;
}

static void handleB(const char *pkt) { // core config
  if (strlen(pkt) < 4) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  float sp, db;
  unsigned long wt, maxOpen;
  if (sscanf(pkt + 2, "%f,%f,%lu,%lu", &sp, &db, &wt, &maxOpen) != 4 ||
      sp < 0.0f || db <= 0.0f || wt > 60000UL || maxOpen > 60000UL) {
    Serial2.println("BB_ERROR:parse");
    return;
  }
  ctrl->configureCore(sp, db, (uint32_t)wt, (uint32_t)maxOpen);
  bbSaveEeprom(bbLox, bbFuel);
}

static void handleV(const char *pkt) { // auto-vent config
  if (strlen(pkt) < 4) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  float trig;
  int autoOn;
  if (sscanf(pkt + 2, "%f,%d", &trig, &autoOn) != 2 ||
      (autoOn != 0 && autoOn != 1)) {
    Serial2.println("BB_ERROR:parse");
    return;
  }
  ctrl->configureVent(trig, autoOn != 0);
  bbSaveEeprom(bbLox, bbFuel);
}

static void handleD(const char *pkt) { // predictive close-delay config
  if (strlen(pkt) < 3) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  unsigned long closeDelayMs;
  char trailing;
  if (sscanf(pkt + 2, "%lu%c", &closeDelayMs, &trailing) != 1 ||
      closeDelayMs > 1000UL) {
    Serial2.println("BB_ERROR:parse");
    return;
  }
  ctrl->configurePredictiveClose((uint32_t)closeDelayMs);
  bbSaveEeprom(bbLox, bbFuel);
}

static void handleM(const char *pkt) { // massflow config
  if (strlen(pkt) < 4) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  float mdot, spMin, spMax, gain, rho;
  int on;
  if (sscanf(pkt + 2, "%f,%f,%f,%f,%f,%d", &mdot, &spMin, &spMax, &gain, &rho,
             &on) != 6 ||
      (on != 0 && on != 1) || spMin > spMax || rho < 0.0f) {
    Serial2.println("BB_ERROR:parse");
    return;
  }
  ctrl->configureMdot(mdot, spMin, spMax, gain, rho, on != 0);
  bbSaveEeprom(bbLox, bbFuel);
}

static void handleLowerB(const char *pkt) { // enable/disable sustain
  if (strlen(pkt) < 3) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  char stCh = pkt[2];
  if (stCh == '1') {
    if (!gArmed) {
      Serial2.println("BB_ERROR:not_armed");
      return;
    }
    if (!hasFreshV2Pt()) {
      Serial2.println("BB_ERROR:pt_stale");
      return;
    }
    if (!ptPsiSettled()) {
      Serial2.println("BB_ERROR:pt_settling");
      return;
    }
    ctrl->enableSustain();
  } else if (stCh == '0') {
    ctrl->disableSustain();
  } else {
    Serial2.println("BB_ERROR:bad_arg");
  }
}

static void handleLowerE(const char *pkt) { // predictive cutoff enable
  if (strlen(pkt) != 3) {
    Serial2.println("BB_ERROR:parse");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  if (pkt[2] == '1') {
    if (!gArmed) {
      Serial2.println("BB_ERROR:not_armed");
      return;
    }
    ctrl->setPredictiveEnabled(true);
  } else if (pkt[2] == '0') {
    ctrl->setPredictiveEnabled(false);
  } else {
    Serial2.println("BB_ERROR:bad_arg");
  }
}

static void handleLowerV(const char *pkt) { // manual vent
  if (strlen(pkt) < 3) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  char stCh = pkt[2];
  if (stCh == '1') {
    if (!gArmed) {
      Serial2.println("BB_ERROR:not_armed");
      return;
    }
    ctrl->manualVent();
  } else if (stCh == '0') {
    ctrl->manualVentClose(false);
  } else {
    Serial2.println("BB_ERROR:bad_arg");
  }
}

static void handleLowerX(const char *pkt) { // latched abort
  if (strlen(pkt) < 2) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  ctrl->latchAbort();
}

static void applyPtTareOffset(uint8_t ch, float offsetPsi, char side) {
  ptTarePsiOffset[ch] = offsetPsi;
  resetMedianFilter(ch);
  ptTareSaveEeprom();

  char detail[48];
  snprintf(detail, sizeof(detail), "ch=%u,offset=%.3f", ch, offsetPsi);
  bbEmit("PT_TARE", side, detail);
}

static void handleT(const char *pkt) {
  if (!pkt || pkt[0] != 'T') {
    Serial2.println("PT_ERROR:parse");
    return;
  }

  const char *body = pkt + 1;
  if (body[0] == 'z' && body[1] == '\0') {
    memset(ptTarePsiOffset, 0, sizeof(ptTarePsiOffset));
    for (uint8_t ch = 0; ch < NUM_PT_CHANNELS; ch++)
      resetMedianFilter(ch);
    ptTareSaveEeprom();
    bbEmit("PT_TARE", '-', "clear all");
    return;
  }

  if (body[0] == 'L' && body[1] == '\0') {
    if (!hasFreshV2Pt()) {
      Serial2.println("PT_ERROR:no_data");
      return;
    }
    applyPtTareOffset(BB_LOX_PT_CH, ptSignalToPsi(v2PtData[BB_LOX_PT_CH]), 'L');
    return;
  }

  if (body[0] == 'F' && body[1] == '\0') {
    if (!hasFreshV2Pt()) {
      Serial2.println("PT_ERROR:no_data");
      return;
    }
    applyPtTareOffset(BB_FUEL_PT_CH, ptSignalToPsi(v2PtData[BB_FUEL_PT_CH]), 'F');
    return;
  }

  unsigned ch = 0;
  float offsetPsi = 0.0f;
  if (sscanf(body, "%u,%f", &ch, &offsetPsi) != 2 || ch >= NUM_PT_CHANNELS ||
      !isfinite(offsetPsi)) {
    Serial2.println("PT_ERROR:parse");
    return;
  }

  char side = '-';
  if (ch == BB_LOX_PT_CH)
    side = 'L';
  else if (ch == BB_FUEL_PT_CH)
    side = 'F';
  applyPtTareOffset((uint8_t)ch, offsetPsi, side);
}

// ── 1 Hz BB summary heartbeat ────────────────────────────────────────────
static const char *stateStr(BBState s) {
  switch (s) {
  case BBState::DISABLED:
    return "OFF";
  case BBState::SUSTAIN:
    return "SUS";
  case BBState::AUTO_VENT:
    return "AV";
  case BBState::ABORT:
    return "ABT";
  }
  return "??";
}

static void printBbHeartbeat(const BBController &c) {
  Serial2.print("BB:");
  Serial2.print(c.busId());
  Serial2.print(':');
  Serial2.print(stateStr(c.state()));
  Serial2.print(':');
  Serial2.print(c.isPressOpen() ? 1 : 0);
  Serial2.print(':');
  Serial2.print(c.isVentOpen() ? 1 : 0);
  Serial2.print(':');
  Serial2.println(c.lastPressure(), 1);
}

// BBD:<side>:<state>:<press01>:<pressure_psi>:<rate_psi_s>:
//     <projected_close_psi>:<deadband_high_psi>:<close_delay_ms>:
//     <total_horizon_ms>:<rate_valid01>:<predictive_enabled01>
// Kept separate from BB: so existing GC heartbeat parsers remain compatible.
static void printBbDebug(const BBController &c) {
  const float hi = c.config().setpoint_psi + c.config().deadband_psi * 0.5f;
  Serial2.print("BBD:");
  Serial2.print(c.busId());
  Serial2.print(':');
  Serial2.print(stateStr(c.state()));
  Serial2.print(':');
  Serial2.print(c.isPressOpen() ? 1 : 0);
  Serial2.print(':');
  Serial2.print(c.lastPressure(), 2);
  Serial2.print(':');
  Serial2.print(c.pressureRate(), 2);
  Serial2.print(':');
  Serial2.print(c.projectedPressure(), 2);
  Serial2.print(':');
  Serial2.print(hi, 2);
  Serial2.print(':');
  Serial2.print(c.config().close_delay_ms);
  Serial2.print(':');
  Serial2.print(c.predictionHorizonMs(), 1);
  Serial2.print(':');
  Serial2.print(c.pressureRateValid() ? 1 : 0);
  Serial2.print(':');
  Serial2.println(c.predictiveEnabled() ? 1 : 0);
}

// ── GC link status line (1 Hz, alongside the BB heartbeat) ───────────────
// LINK:<armed01>:<lost01>:<silent_ms>
// <armed01> is the single most important field: 0 means GC has never sent a
// heartbeat this boot and NOTHING here is protecting the stand.
static void printLinkStatus(uint32_t now) {
  Serial2.print("LINK:");
  Serial2.print(gcWatchdogArmed ? 1 : 0);
  Serial2.print(':');
  Serial2.print(gcLinkLost ? 1 : 0);
  Serial2.print(':');
  Serial2.println(gcWatchdogArmed ? (now - lastGcRxMs) : 0UL);
}

// ── GC primary-link watchdog ─────────────────────────────────────────────
// Two stages, mirroring the fail-safe philosophy of the V2 PT watchdog:
//
//   stage 1 (COMMS_LOSS_MS)   bang-bang control off, BB valves driven closed
//   stage 2 (COMMS_DISARM_MS) full disarm, byte-for-byte what operator 'r' does
//
// Recovery never restarts control on its own. As with PT_STALE, a healthy link
// coming back only clears the latch; the operator must re-arm and re-issue
// b<side>1. Anything else would let a flapping link cycle live valves.
static void serviceGcLinkWatchdog(uint32_t now) {
  if (!gcWatchdogArmed)
    return;

  const uint32_t silentMs = now - lastGcRxMs;

  if (silentMs < COMMS_LOSS_MS) {
    if (gcLinkLost) {
      gcLinkLost = false;
      gcDisarmDone = false;
      bbEmit("COMMS_OK", '-', "GC link restored; re-arm and re-enable to resume");
    }
    return;
  }

  if (!gcLinkLost) {
    gcLinkLost = true;
    bbEmit("COMMS_LOSS", '-', "GC link silent; bang-bang forced safe");
  }

  // forceSafe() drives press and vent closed unconditionally and is a no-op
  // once they already are (_setPress/_setVent early-return when unchanged), so
  // calling it every tick costs nothing and emits nothing after the first.
  // Running it unconditionally rather than only from SUSTAIN/AUTO_VENT also
  // sweeps up a BB channel an operator had opened by hand while BB was off.
  //
  // ABORT is deliberately exempt: it parks the vent OPEN, which is the safe
  // state for the over-pressure that latched it, and force-safing would close
  // that vent on a stand nobody can currently talk to. The latch still clears
  // only on a real disarm — including the stage-2 disarm below.
  if (bbLox.state() != BBState::ABORT)
    bbLox.forceSafe();
  if (bbFuel.state() != BBState::ABORT)
    bbFuel.forceSafe();

  if (silentMs >= COMMS_DISARM_MS && !gcDisarmDone) {
    gcDisarmDone = true;
    bbEmit("COMMS_DISARM", '-', "no GC link for 10s; disarming");
    digitalWrite(PIN_ARM, LOW);
    digitalWrite(PIN_DISARM, HIGH);
    gArmed = false;
    bbLox.forceSafe();
    bbFuel.forceSafe();
    sh.cancelExecution();
    sh.setAllChannelsOff();
  }
}

// Periodic telemetry is best-effort. Unlike command responses and BB events,
// it must never block the main loop waiting for UART buffer space: doing that
// can prevent us from reading the disable/disarm command which clears a live
// control state. The cadence below leaves ample idle time on the RS-485 bus.
static bool tryWriteTelemetryFrame(const char *lctcPacket,
                                   const char *sPacket,
                                   const char *ptPacket,
                                   const char *ptPsiPacket) {
  const size_t frameLen = strlen(lctcPacket) + strlen(sPacket) +
                          strlen(ptPacket) + strlen(ptPsiPacket);
  const int available = Serial2.availableForWrite();
  if (available < 0 || static_cast<size_t>(available) <
                           frameLen + TX_PRIORITY_RESERVE) {
    return false;
  }

  Serial2.print(lctcPacket);
  Serial2.print(sPacket);
  Serial2.print(ptPacket);
  Serial2.print(ptPsiPacket);
  return true;
}

// Compact debug helper for validating UART framing bytes without flooding logs.
static void printPacketHexBrief(const char *label, const char *packet,
                                size_t maxLen) {
  if (!packet) {
    return;
  }

  const size_t n = strnlen(packet, maxLen);
  Serial.print("HEX ");
  Serial.print(label);
  Serial.print(" len=");
  Serial.print(n);
  Serial.print(" head=");

  const size_t headCount = (n < 8) ? n : 8;
  for (size_t i = 0; i < headCount; i++) {
    if (i)
      Serial.print(' ');
    if ((uint8_t)packet[i] < 0x10)
      Serial.print('0');
    Serial.print((uint8_t)packet[i], HEX);
  }

  Serial.print(" tail=");
  const size_t tailCount = (n < 3) ? n : 3;
  for (size_t i = n - tailCount; i < n; i++) {
    if (i != n - tailCount)
      Serial.print(' ');
    if ((uint8_t)packet[i] < 0x10)
      Serial.print('0');
    Serial.print((uint8_t)packet[i], HEX);
  }
  Serial.println();
}

static void printBytesHexBrief(const char *label, const uint8_t *data,
                               size_t len) {
  if (!data) {
    return;
  }

  Serial.print("HEX ");
  Serial.print(label);
  Serial.print(" len=");
  Serial.print(len);
  Serial.print(" head=");

  const size_t headCount = (len < 8) ? len : 8;
  for (size_t i = 0; i < headCount; i++) {
    if (i)
      Serial.print(' ');
    if (data[i] < 0x10)
      Serial.print('0');
    Serial.print(data[i], HEX);
  }

  Serial.print(" tail=");
  const size_t tailCount = (len < 3) ? len : 3;
  for (size_t i = len - tailCount; i < len; i++) {
    if (i != len - tailCount)
      Serial.print(' ');
    if (data[i] < 0x10)
      Serial.print('0');
    Serial.print(data[i], HEX);
  }
  Serial.println();
}

void setup() {
  // Primary RS-485 (GC) — Serial2 = LPUART4, pins 7(RX)/8(TX).
  Serial2.begin(SERIAL_BAUD_RATE);
  Serial2.setTimeout(100);
  static uint8_t rxBuf[RX_BUF_SIZE];
  Serial2.addMemoryForRead(rxBuf, RX_BUF_SIZE);
  static uint8_t txBuf[TX_BUF_SIZE];
  Serial2.addMemoryForWrite(txBuf, TX_BUF_SIZE);

  // Secondary crossover (direct TTL UART from V2 Serial6: V2 pin 24 → V1 pin
  // 21, V2 pin 25 ← V1 pin 20). RS-485 transceiver bypassed on this bus.
  Serial5.begin(SERIAL_BAUD_RATE);
  Serial5.setTimeout(100);
  static uint8_t rxBufXover[RX_BUF_SIZE];
  Serial5.addMemoryForRead(rxBufXover, RX_BUF_SIZE);
  static uint8_t txBufXover[TX_BUF_SIZE];
  Serial5.addMemoryForWrite(txBufXover, TX_BUF_SIZE);

  Serial.begin(SERIAL_BAUD_RATE); // Debugging via serial monitor

  SPI.begin();
  SPI.setClockDivider(4);
  SPI1.begin();
  SPI1.setClockDivider(4);
  Wire2.begin();

  sh.setup();
  sScanner.setup();
  fScanner.setup();

  pinMode(PIN_DISARM, OUTPUT);
  pinMode(PIN_ARM, OUTPUT);
  digitalWrite(PIN_DISARM, HIGH);

  // Wire BB IO and load persisted config. Controllers come up DISABLED
  // regardless of what EEPROM contained — config is restored but the state
  // machine starts safe.
  bbLox.bindIO(bbSetChannel, bbEmit);
  bbFuel.bindIO(bbSetChannel, bbEmit);
  bbLox.forceSafe();
  bbFuel.forceSafe();
  bbLoadEeprom(bbLox, bbFuel);
  ptTareLoadEeprom();
  Serial2.print("PT_TARE:ch0=");
  Serial2.print(ptTarePsiOffset[0], 3);
  Serial2.print(",ch1=");
  Serial2.println(ptTarePsiOffset[1], 3);

  Serial2.println("Panda Initialized!");
}

void loop() {
  char idChar;

  // ── Secondary bus: drain PT forwards from V2 ────────────────────────
  thXover.poll();
  if (thXover.isPacketReady()) {
    char *xPacket = thXover.takePacket();
    static uint32_t lastXoverDiagMs = 0;
    const uint32_t now = millis();
    if (DEBUG_PACKET && now - lastXoverDiagMs >= 1000UL) {
      const size_t xLen = thXover.getLastPacketLen();
      Serial.print("XOVER delim=");
      Serial.print(thXover.didLastPacketEndWithDelimiter() ? 1 : 0);
      Serial.print(' ');
      printBytesHexBrief("XOVER",
                         reinterpret_cast<const uint8_t *>(xPacket), xLen);
      lastXoverDiagMs = now;
    }
    if (xPacket[0] == PT_IDENTIFIER) {
      parseV2PtPacket(xPacket);
    }
    size_t n = strnlen(xPacket, RX_BUF_SIZE);
    memset(xPacket, 0, n);
  }

  // ── Primary bus: commands from GC ───────────────────────────────────
  th.poll();
  if (th.isPacketReady()) {
    char *rxPacket = th.takePacket();
    // Any complete line proves GC and the wire are alive — a malformed one
    // included. Refresh before dispatch so even a rejected command counts.
    lastGcRxMs = millis();
    if (DEBUG_PACKET)
      Serial.println(rxPacket);
    idChar = rxPacket[0];

    if (idChar == 's') {
      sh.setCommand(rxPacket);
    } else if (idChar == 'S') {
      char channelChar = rxPacket[1];
      char stateChar = rxPacket[2];
      unsigned channel, state;
      if (channelChar >= '0' && channelChar <= '9')
        channel = channelChar - '0';
      else
        channel = 10 + (toupper(channelChar) - 'A');
      state = stateChar - '0';

      if (channel >= 1 && channel <= NUM_DC_CHANNELS) {
        if (isBbOwned(channel)) {
          Serial2.print("CMD_ERROR: chan ");
          Serial2.print(channel);
          Serial2.println(" owned by BB");
        } else {
          sh.channelArr[channel - 1].setState(state);
          Serial2.print("Solenoid Command: ");
          Serial2.print(channel);
          Serial2.print(" | ");
          Serial2.println(state);
        }
      }
    } else if (idChar == 'a') {
      digitalWrite(PIN_DISARM, LOW);
      digitalWrite(PIN_ARM, HIGH);
      gArmed = true;
      Serial2.println("Arming!");
    } else if (idChar == 'r') {
      digitalWrite(PIN_ARM, LOW);
      digitalWrite(PIN_DISARM, HIGH);
      gArmed = false;
      bbLox.forceSafe();
      bbFuel.forceSafe();
      sh.cancelExecution();
      sh.setAllChannelsOff();
      Serial2.println("Disarming!");
      Serial2.println("SEQ_ABORT: Outputs de-energized");
      if (sh.hasSequence()) {
        Serial2.print("SEQ_READY:");
        Serial2.println(sh.getLastCommand());
      }
    } else if (idChar == 'f') {
      if (!sh.hasSequence()) {
        Serial2.println("SEQ_ERROR: No sequence loaded");
      } else {
        Serial2.print("SEQ_EXEC_START:count=");
        Serial2.print(sh.getNumCommands());
        const char *lastCmd = sh.getLastCommand();
        if (lastCmd && lastCmd[0] != '\0') {
          Serial2.print(",raw=");
          Serial2.println(lastCmd);
        } else {
          Serial2.println();
        }
        sh.execute(true);
        Serial2.println("Firing sequence!");
      }
    }
    // Bang-bang commands
    else if (idChar == 'B')
      handleB(rxPacket);
    else if (idChar == 'D')
      handleD(rxPacket);
    else if (idChar == 'V')
      handleV(rxPacket);
    else if (idChar == 'M')
      handleM(rxPacket);
    else if (idChar == 'b')
      handleLowerB(rxPacket);
    else if (idChar == 'e')
      handleLowerE(rxPacket);
    else if (idChar == 'v')
      handleLowerV(rxPacket);
    else if (idChar == 'x')
      handleLowerX(rxPacket);
    else if (idChar == 'T')
      handleT(rxPacket);
    else if (idChar == GC_HEARTBEAT_IDENTIFIER) {
      // Liveness only, and deliberately silent: at 5 Hz an acknowledgement per
      // beat would put avoidable traffic on a half-duplex bus this firmware
      // works hard to keep idle. The 1 Hz LINK: line is the acknowledgement.
      if (!gcWatchdogArmed) {
        gcWatchdogArmed = true;
        bbEmit("COMMS_WD_ARM", '-', "GC heartbeat seen; link watchdog active");
      }
    }

    size_t n = strnlen(rxPacket, 256);
    memset(rxPacket, 0, n);
  }

  sh.update();

  // Service before BB runs, so no controller actuates on this tick on the
  // strength of a command from a link that is already gone.
  serviceGcLinkWatchdog(millis());

  // ── Bang-bang step ───────────────────────────────────────────────────
  // Loss of the V2 crossover must not leave a valve controlled indefinitely
  // from a stale pressure sample. A fresh frame does not auto-restart control;
  // the operator must explicitly enable sustain again. ABORT remains latched:
  // only an operator disarm may clear it, regardless of PT-link health.
  if (!hasFreshV2Pt()) {
    if (bbLox.state() == BBState::SUSTAIN ||
        bbLox.state() == BBState::AUTO_VENT) {
      bbEmit("PT_STALE", 'L', "V2 PT timeout; forcing safe");
      bbLox.forceSafe();
    }
    if (bbFuel.state() == BBState::SUSTAIN ||
        bbFuel.state() == BBState::AUTO_VENT) {
      bbEmit("PT_STALE", 'F', "V2 PT timeout; forcing safe");
      bbFuel.forceSafe();
    }
  }
  const uint32_t pressureSampleMs = hasFreshV2Pt() ? lastV2PtMs : 0;
  bbLox.update(gArmed, ptPsiSettled(), pressureSampleMs);
  bbFuel.update(gArmed, ptPsiSettled(), pressureSampleMs);

  // ========== DAQ ==========
  sScanner.update();
  fScanner.update();

  static uint32_t lastTelemetryMs = 0;
  static uint32_t lastHeartbeatMs = 0;
  static uint32_t lastBbDebugMs = 0;
  static uint32_t lastHexDiagMs = 0;
  const uint32_t now = millis();

  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    // Advance the schedule even if this frame is dropped. Retrying immediately
    // would recreate the same tight-loop TX starvation this guard prevents.
    lastTelemetryMs = now;

    float sData[NUM_DC_CHANNELS] = {0};
    float lctcData[NUM_LC_CHANNELS + NUM_TC_CHANNELS] = {0};
    sScanner.getSOutput(sData);
    fScanner.getLCTCOutput(lctcData);

    char sPacket[512], lctcPacket[512], ptPacket[512], ptPsiPacket[512];
    const bool frameValid =
        th.toCSVRow(sData, S_IDENTIFIER, NUM_DC_CHANNELS, sPacket,
                    sizeof(sPacket), 5) &&
        th.toCSVRow(lctcData, LCTC_IDENTIFIER,
                    NUM_TC_CHANNELS + NUM_LC_CHANNELS, lctcPacket,
                    sizeof(lctcPacket), 5) &&
        th.toCSVRow(v2PtData, PT_IDENTIFIER, NUM_PT_CHANNELS, ptPacket,
                    sizeof(ptPacket), 5) &&
        th.toCSVRow(v2PtPsiData, PT_PSI_IDENTIFIER, NUM_PT_CHANNELS,
                    ptPsiPacket, sizeof(ptPsiPacket), 5);

    if (frameValid)
      tryWriteTelemetryFrame(lctcPacket, sPacket, ptPacket, ptPsiPacket);

    if (DEBUG_PACKET && frameValid && now - lastHexDiagMs >= 1000UL) {
      printPacketHexBrief("LCTC", lctcPacket, sizeof(lctcPacket));
      printPacketHexBrief("S", sPacket, sizeof(sPacket));
      printPacketHexBrief("PT", ptPacket, sizeof(ptPacket));
      lastHexDiagMs = now;
    }
  }

  if (now - lastHeartbeatMs >= BB_HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    // Heartbeats are small and infrequent. Only send them when they cannot
    // consume the space reserved for control responses/events.
    // +128 covers both BB heartbeat lines plus the LINK: status line.
    if (Serial2.availableForWrite() >=
        static_cast<int>(TX_PRIORITY_RESERVE + 128)) {
      printBbHeartbeat(bbLox);
      printBbHeartbeat(bbFuel);
      printLinkStatus(now);
    }
  }

  if (now - lastBbDebugMs >= BB_DEBUG_INTERVAL_MS) {
    lastBbDebugMs = now;
    if (Serial2.availableForWrite() >=
        static_cast<int>(TX_PRIORITY_RESERVE + 192)) {
      printBbDebug(bbLox);
      printBbDebug(bbFuel);
    }
  }
}
