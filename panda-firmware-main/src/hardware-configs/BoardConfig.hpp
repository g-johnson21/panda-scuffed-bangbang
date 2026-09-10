#pragma once
#include <SPI.h>
#include <Arduino.h>

/**
 * Configuration file
 * Includes DC channel and Bang-Bang implementation, time values, and conversion constants
 */

 // =================== SPI & ADC configurations ================= //

static const SPISettings SPISettingsDefault(20000000, MSBFIRST, SPI_MODE0);
static constexpr unsigned int T_MUX_SETTLE_US = 500;
static constexpr unsigned int T_CONV_US = 1000;

// =================== Serial configurations ================= //

static constexpr bool DEBUG_F_ADC = false;
static constexpr bool DEBUG_BB = false;
static constexpr bool DEBUG_PACKET = false;

static constexpr char S_IDENTIFIER = 's';
static constexpr char LCTC_IDENTIFIER = 't';
static constexpr char PT_IDENTIFIER = 'p'; // PT CSV id (forwarded from V2)
static constexpr char PT_PSI_IDENTIFIER = 'P'; // Scaled PT PSI row id (V1-derived)

static constexpr unsigned int SERIAL_BAUD_RATE = 460800; // Originally 115200
static constexpr unsigned int SERIAL_TIMEOUT = 2000;
// static constexpr unsigned int SERIAL_WRITE_DELAY = 100; // Microseconds to wait between writes to prevent overwhelming the serial bus

static constexpr unsigned int PACKET_IDLE_MS = 100;
static constexpr unsigned int PULSE_DURATION = 500;
static constexpr size_t RX_BUF_SIZE = 256;
// Keep the half-duplex command bus idle most of the time. Periodic telemetry
// must never continuously refill the UART TX buffer or GC cannot get a command
// (most importantly a disable/disarm) onto the wire.
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 50;       // 20 Hz
static constexpr uint32_t BB_HEARTBEAT_INTERVAL_MS = 1000;  // 1 Hz
static constexpr uint32_t BB_DEBUG_INTERVAL_MS = 100;       // 10 Hz
static constexpr size_t TX_PRIORITY_RESERVE = 256;

// ============== GC primary-link (Serial2) watchdog ============= //
// The primary RS-485 link only carries GC->V1 traffic when the operator acts,
// so silence by itself is NOT evidence of a dead link. GC must send a periodic
// heartbeat ('h') for the board to tell "quiet" apart from "gone".
//
// The watchdog is HEARTBEAT-GATED: it stays dormant until the first 'h' of the
// boot arrives, then latches armed until reset. A GC that never sends
// heartbeats therefore behaves exactly as it did before this feature existed
// (no protection, but no nuisance disarms either); a GC that does send them
// gets full protection from its first beat onward. Whether the watchdog is
// actually armed is published every second on the LINK: telemetry line, so an
// un-armed watchdog can never be mistaken for a healthy one.
static constexpr char GC_HEARTBEAT_IDENTIFIER = 'h';
// GC should send 'h' at 5 Hz. Three missed beats trip stage 1; the value is
// the direct analogue of BB_PT_STALE_MS on the V2 crossover.
static constexpr uint32_t COMMS_LOSS_MS = 600;      // stage 1: bang-bang safe
static constexpr uint32_t COMMS_DISARM_MS = 10000;  // stage 2: full disarm
static_assert(COMMS_LOSS_MS < COMMS_DISARM_MS,
              "BB must be forced safe before the disarm stage runs");

static constexpr uint8_t NUM_MAX_COMMANDS = 32;
static constexpr uint8_t DATA_DECIMALS = 6; // Number of decimal places in telemetry data

static constexpr uint8_t NUM_DC_CHANNELS = 12;
static constexpr uint8_t NUM_PT_CHANNELS = 2;
static constexpr uint8_t NUM_LC_CHANNELS = 6;
static constexpr uint8_t NUM_TC_CHANNELS = 6;

static constexpr uint8_t PACKET_SIZE = NUM_DC_CHANNELS + NUM_PT_CHANNELS + NUM_LC_CHANNELS + NUM_TC_CHANNELS;
static constexpr size_t TX_BUF_SIZE = 2048;

// Conversion constants
static constexpr float tcConstant = 2217.294;
static constexpr float tcOffset = 160;
static constexpr float sConstant = 0.5;
// V2 PT stream arrives as raw shunt voltage. Convert on V1: volts -> mA -> psi,
// used for secondary scaled telemetry row.
static constexpr float PT_SHUNT_OHMS = 47.0f;      // 4-20 mA into 47 ohm = 1-5 V
static constexpr float PT_ZERO_MA = 4.0f;
static constexpr float PT_SPAN_MA = 16.0f;          // 20 - 4
static constexpr float PT_FULL_SCALE_PSI = 1500.0f * 0.9748f; // Sensor rating times scale

static constexpr float tcOffsets[NUM_TC_CHANNELS] = {
    -0.07429,
    -0.06889,
    -0.07387,
    -0.0786,
    -0.06998,
    -0.07754
};

// =================== Bang-Bang configuration =================== //
// Bang-bang runs on V1 (DC channels functional here) using PT readings
// forwarded from V2 over the secondary RS-485 crossover.

// Sentinel values for unset hardware channels.
static constexpr uint8_t BB_DC_CH_UNSET = 0;     // DC channels are 1-indexed
static constexpr uint8_t BB_PT_CH_UNSET = 0xFF;  // PT channels are 0-indexed

// PT indices into the V2-forwarded ptData[] array (0-indexed).
static constexpr uint8_t BB_LOX_PT_CH  = 0;
static constexpr uint8_t BB_FUEL_PT_CH = 1;

// DC press-solenoid channels (1-indexed; maps to SequenceHandler::channelArr[ch-1]).
static constexpr uint8_t BB_LOX_DC_CH  = 1;
static constexpr uint8_t BB_FUEL_DC_CH = 2;

// DC vent-solenoid channels (1-indexed). Set to BB_DC_CH_UNSET to disable
// auto-vent / abort for that side; those commands will be rejected with
// EVT:...:AV_NO_HW until a real channel is configured here.
// TODO(user): set these to the actual vent DC channels on your board.
static constexpr uint8_t BB_LOX_VENT_DC_CH  = 3;
static constexpr uint8_t BB_FUEL_VENT_DC_CH = 4;

// Venturi PT indices for mass-flow calculation (0-indexed into v2PtData[]).
// BB_PT_CH_UNSET disables mass-flow correction for that side regardless of
// the `mdot_enabled` config flag — no fake numbers will drive the loop.
// TODO(user): set these once the additional 4 venturi PTs are installed on V2.
static constexpr uint8_t BB_LOX_VENTURI_UP_PT  = BB_PT_CH_UNSET;
static constexpr uint8_t BB_LOX_VENTURI_DN_PT  = BB_PT_CH_UNSET;
static constexpr uint8_t BB_FUEL_VENTURI_UP_PT = BB_PT_CH_UNSET;
static constexpr uint8_t BB_FUEL_VENTURI_DN_PT = BB_PT_CH_UNSET;

// Sanity bounds — BB disables if PT reading falls outside this range.
static constexpr float BB_PRESSURE_MIN_PSI = -50.0f;
static constexpr float BB_PRESSURE_MAX_PSI = 4000.0f;
// V2 sends PT data at ~250 Hz. ~50 ms without a complete frame force-safes any
// active BB controller; operator must explicitly re-enable after data resumes.
static constexpr uint32_t BB_PT_STALE_MS = 50;
// BB consumes the rolling-median PSI signal. A monotonic ramp is delayed by
// half this window, so predictive cutoff compensates that measured delay in
// addition to the configured mechanical valve-close delay.
static constexpr uint8_t PT_PSI_MEDIAN_WINDOW = 75;

// Mass-flow correction update cadence (ms between setpoint nudges).
static constexpr uint32_t BB_MDOT_UPDATE_MS = 500;

// Predictive press-valve cutoff. Pressure rate is low-pass filtered so one
// noisy PT derivative does not command an early close. The mechanical delay
// itself is persisted per side in BBConfig and defaults to 15 ms.
static constexpr float BB_PRESSURE_RATE_ALPHA = 0.20f;
static constexpr uint32_t BB_RATE_RESET_MS = 100;

// Venturi CdA (discharge coefficient × effective throat area) in m².
// Single value covers both LOX and Fuel venturis — update here if they
// are ever sized differently. With this form of the mass-flow equation
// upstream/throat areas are not used separately.
//
//   m_dot [kg/s] = CdA · sqrt( 2 · ρ · ΔP )
//   ΔP [Pa]      = (P_up − P_dn) [psi] · PSI_TO_PA
static constexpr float BB_VENTURI_CDA_M2 = 3.22e-5f;
static constexpr float PSI_TO_PA         = 6894.757f;

// EEPROM layout for BB config persistence. Magic bumped when BBConfig
// struct layout changed; old EEPROM contents ignored on mismatch.
static constexpr uint16_t BB_EEPROM_MAGIC = 0xBB45;
static constexpr int      BB_EEPROM_ADDR  = 0;

// PT tare offsets (PSI subtracted after V2→PSI conversion). Separate EEPROM
// block so layout changes do not collide with BB config.
static constexpr uint16_t PT_TARE_EEPROM_MAGIC = 0x5441; // "TA"
static constexpr int      PT_TARE_EEPROM_ADDR  = 256;

static constexpr uint8_t NUM_ACTUATORS = NUM_DC_CHANNELS;
