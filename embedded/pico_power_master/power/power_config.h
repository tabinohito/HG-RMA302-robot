#pragma once

// RP2040 pin numbers. ESP32's own buzzer is GPIO27 on the other processor.
#define POWER_I2C_ID             i2c1
#define POWER_I2C_SDA_PIN        26
#define POWER_I2C_SCL_PIN        27
#define POWER_WS2812_PIN         28
#define POWER_WS2812_COUNT       30u
#define POWER_WS2812_RESET_US    300u
#define POWER_WS2812_REFRESH_MS  250u
#define POWER_RELAY_PIN          2
#define POWER_BUTTON_PIN         5
#define POWER_SHUNT_UOHM         100000u // User-confirmed R100 = 0.1 ohm.
#define POWER_I2C_TIMEOUT_US     2000u
#define POWER_SAMPLE_MS          100u
#define POWER_DISPLAY_MS         200u

// User-selected separator: the LOWER center colon dot, between digits 2/3.
// Real-board probes confirmed slot 3 bit 7 = lower dot, slot 2 bit 7 = upper.
// This is the zero-based wire slot, not the decimal position in the number.
#define POWER_DECIMAL_SLOT       3u

// Desired panel controls, from the right: ON/OFF, START, HOME, OK.
// ON/OFF is GP5. Assign the three existing UART key masks to named roles.
// Photo silkscreen: START=K1, HOME=K2, OK=K3. The label-to-GPIO assignment
// below follows the existing key order and still needs a physical key check.
#define POWER_KEY_START_MASK     1u // ESP32 GPIO22 -> voltage
#define POWER_KEY_HOME_MASK      2u // ESP32 GPIO21 -> current
#define POWER_KEY_OK_MASK        4u // ESP32 GPIO19 -> power

// Keep all master-triggered sounds muted until explicitly enabled again.
#ifndef POWER_BUZZER_ENABLE
#define POWER_BUZZER_ENABLE      0
#endif
