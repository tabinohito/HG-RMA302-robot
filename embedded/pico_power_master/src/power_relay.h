#pragma once
#include "hardware/gpio.h"
#include "power/power_config.h"

// User-confirmed active High. Keep OFF in both controller and flash bridge.
static inline void power_relay_init_off(void) {
    gpio_init(POWER_RELAY_PIN);
    gpio_put(POWER_RELAY_PIN, 0);
    gpio_set_dir(POWER_RELAY_PIN, GPIO_OUT);
}
