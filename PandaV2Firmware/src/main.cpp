#include <Arduino.h>
#include <SPI.h>

#include "BoardConfig.h"
#include "pins.h"

#include <MCP3561RT.h>

#include "CommsHandler.h"
#include "Scanner.h"
#include "SensorConfig.h"

// ─────────────────────────────────────────────────────────────────────────────
// PandaV2 — PT telemetry only.
//
// This firmware has been deliberately stripped to the minimum needed to do one
// job: read LOX/FUEL PT channels (mux A ch 0–1) off ADC1 and emit the CSV
// row on both UARTs.
//
//   Primary  (Serial7, RS-485) → GC: PT CSV row at TELEMETRY_INTERVAL_MS.
//   Crossover (Serial6, TTL)   → V1: same row, same cadence.
//
// Everything else — arming, sequences, bang-bang, LC/TC/current sensing,
// power monitoring, board temp, commands — has been removed. V2's DC outputs
// and bus-2 transceiver are broken in hardware; V1 owns the control loop.
// ─────────────────────────────────────────────────────────────────────────────

// ── Hardware ───────────────────────────────────────────────────────────────
MCP3561RT adc1(PIN_ADC1_CS, PIN_ADC1_IRQ, SPI, SPI_ADC_SETTINGS, 1.25f);

CommsHandler comms (Serial7, PIN_RS485_1_DE);   // RS-485 → GC
CommsHandler comms2(Serial6);                   // direct TTL → V1

// ── Buffers ────────────────────────────────────────────────────────────────
static float muxA_data[NUM_MUX_A_CH] = {0};
static float ptData   [NUM_PT_CH]    = {0};

// Single mux bank on ADC1 CH0 — the PT chain.
static MuxBank adc1Banks[] = {
    {{PIN_MUX_A_S0, PIN_MUX_A_S1, PIN_MUX_A_S2, PIN_MUX_A_S3},
     NUM_MUX_A_CH,
     MCP3561RT::Mux::CH0,
     muxA_data,
     1.25f},
};

static Scanner scanner1(adc1, adc1Banks, 1);

// ── Telemetry ──────────────────────────────────────────────────────────────
static elapsedMillis telemetryTimer;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 4;    // ~250 Hz (2-ch sweep ~3.5 ms)

static void sendPtCsv(CommsHandler& out) {
    char buf[512];
    CommsHandler::toCSVRow(ptData, 'p', NUM_PT_CH, buf, sizeof(buf));
    out.send(buf);
}

// ── Arduino entry points ───────────────────────────────────────────────────
void setup() {
    Serial.begin(DEBUG_BAUD);

    if (CrashReport) {
        Serial.print(CrashReport);
        delay(2000);
    }

    SPI.begin();

    comms.begin(RS485_BAUD);
    comms2.begin(RS485_BAUD);

    // Primary RS-485 (Serial7 = LPUART7): TX differential pair (Y/Z) is
    // swapped on the V2 PCB RJ45. RXINV stays off — RX pair (A/B) is correct.
    // TODO: fix Y/Z routing in next board revision.
    LPUART7_CTRL |= LPUART_CTRL_TXINV;

    // Secondary (Serial6 = LPUART1): bus-2 RS-485 transceiver is bypassed.
    // The board swap only mattered with the transceiver in the path; driving
    // the Teensy pin straight to V1 is standard TTL. Do NOT set TXINV here —
    // it would invert the line polarity and V1 Serial5 would see garbage.

    comms.sendLine("BOOT");
    if (CrashReport) comms.sendLine("WARN:CRASH_DETECTED");

    bool adc1_ok = adc1.begin();
    scanner1.begin();

    comms.sendLine("PANDA_V2_PT_INIT");
    if (!adc1_ok) comms.sendLine("WARN:ADC1_INIT_FAIL");

    Serial.println("PandaV2 PT-only ready");
}

void loop() {
    scanner1.update();

    // Convert mux-A voltages → PSI (convertPT is currently identity; add
    // per-channel calibration in SensorConfig.h when transducer cal is known).
    for (uint8_t i = 0; i < NUM_PT_CH; i++) {
        ptData[i] = convertPT(muxA_data[i], i);
    }

    if (telemetryTimer >= TELEMETRY_INTERVAL_MS) {
        telemetryTimer = 0;
        sendPtCsv(comms);    // → GC
        sendPtCsv(comms2);   // → V1 crossover
    }

    delayMicroseconds(100);
}
