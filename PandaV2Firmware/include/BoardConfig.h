#pragma once
#include <SPI.h>

// SPI
static const SPISettings SPI_ADC_SETTINGS(20000000, MSBFIRST, SPI_MODE0);

// ADC timing
static constexpr uint32_t T_MUX_SETTLE_US = 500;

// Serial
static constexpr uint32_t RS485_BAUD     = 460800;
static constexpr uint32_t DEBUG_BAUD     = 115200;
static constexpr size_t   RS485_RX_BUF   = 512;
static constexpr size_t   RS485_TX_BUF   = 2048;
static constexpr uint32_t PACKET_IDLE_MS = 100;

// PT-only scanner: mux A channels 0–1 (LOX/FUEL press PTs).
static constexpr uint8_t NUM_MUX_A_CH = 2;

// Telemetry output: 5 decimal places, id 'p' (defined inline at call site).
static constexpr uint8_t DATA_DECIMALS = 5;
