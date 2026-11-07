#pragma once

// -----------------------------------------------------------------------------
// UART connection to ESP32
// -----------------------------------------------------------------------------
#define BRIDGE_UART_ID           uart1
#define BRIDGE_UART_TX_PIN       8   // Pico GP8 -> board R / ESP32 RX (ROM + panel replies verified)
#define BRIDGE_UART_RX_PIN       9   // Pico GP9 <- board T / ESP32 TX (boot log verified)
#define BRIDGE_DEFAULT_BAUD      115200u
// Retain early ESP32 boot output until the host opens USB CDC.
// Must be a power of two. Once full, newer bytes are dropped.
#define BRIDGE_UART_RX_BUFFER_SIZE 16384u
// Let the host finish configuring/flushing its newly opened serial port.
#define BRIDGE_CDC_OPEN_DELAY_US 100000u

// Verified direct connections: GP6 -> ESP32 IO0, GP7 -> EN/RESET.
// These pins only drive Low or become unbiased inputs; never drive High.
// 2400 baud selects the bridge's private control console, not the target UART.
#define BRIDGE_ESP_BOOT_PIN       6
#define BRIDGE_ESP_RESET_PIN      7
#define BRIDGE_CONTROL_BAUD       2400u
#define BRIDGE_ESP_RESET_HOLD_US   100000u
#define BRIDGE_ESP_BOOT_HOLD_US    100000u

// -----------------------------------------------------------------------------
// Optional DTR/RTS GPIO output
// -----------------------------------------------------------------------------
// Leave disabled until you have identified how the target board's auto-reset /
// auto-programming transistor circuit expects DTR and RTS electrically.
//
// When enabled:
//   GP2 = DTR state
//   GP3 = RTS state
//
// Set *_INVERT to 1 if the target circuit expects active-low signals.
#define BRIDGE_MODEM_GPIO_ENABLE 0
#define BRIDGE_DTR_PIN           2
#define BRIDGE_RTS_PIN           3
#define BRIDGE_DTR_INVERT        0
#define BRIDGE_RTS_INVERT        0
