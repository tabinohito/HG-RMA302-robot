#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H
#include "panel_test.h"
#include "power_config.h"

enum { POWER_VOLTAGE, POWER_CURRENT, POWER_WATTS };
typedef struct {
    uint32_t bus_uv;
    int64_t current_ua, power_uw;
    int16_t shunt_raw;
} power_measurement_t;

// INA226: bus LSB 1.25mV, signed shunt LSB 2.5uV. No current-register
// calibration dependency; signed current/power are derived directly from R.
static inline power_measurement_t power_measure(uint16_t bus, uint16_t shunt,
                                                uint32_t shunt_uohm) {
    power_measurement_t m = {0};
    m.bus_uv = (uint32_t)bus * 1250u;
    int32_t signed_shunt = shunt & 0x8000u ? (int32_t)shunt - 65536 : shunt;
    m.shunt_raw = (int16_t)signed_shunt;
    if (shunt_uohm) m.current_ua = (int64_t)signed_shunt * 2500000 / shunt_uohm;
    m.power_uw = (int64_t)m.bus_uv * m.current_ua / 1000000;
    return m;
}

typedef struct {
    bool initialized, candidate, stable;
    uint32_t changed_at;
} power_button_t;

// A switch held during boot is a baseline; require release and a fresh press.
static inline bool power_button_sample(power_button_t *b, bool down, uint32_t now) {
    if (!b->initialized) {
        b->initialized = true; b->candidate = b->stable = down; b->changed_at = now;
        return false;
    }
    if (b->candidate != down) { b->candidate = down; b->changed_at = now; }
    if (b->candidate != b->stable && (uint32_t)(now - b->changed_at) >= 25) {
        b->stable = b->candidate;
        return b->stable;
    }
    return false;
}

static inline void power_error_segments(uint8_t segments[4]) {
    for (unsigned i = 0; i < 4; ++i) segments[i] = 0x40;
}

// Render V/A/W in base units with exactly two decimal places: 00.00 or -0.00.
// The physical separator is the lower center colon dot, driven via slot 3.
// Keep the dot fixed; values that round outside -9.99..99.99 display ----.
static inline bool power_format(int64_t micro, uint8_t segments[4]) {
    const uint32_t divisor = 10000;
    bool negative = micro < 0;
    uint64_t magnitude = negative ? (uint64_t)(-(micro + 1)) + 1 : (uint64_t)micro;
    uint64_t scaled = magnitude / divisor + (magnitude % divisor >= divisor / 2);
    if (scaled > (negative ? 999u : 9999u)) {
        power_error_segments(segments); return false;
    }
    if (!scaled) negative = false;
    for (unsigned pos = 4; pos-- > 0;) {
        segments[pos] = (uint8_t)panel_glyph((char)('0' + scaled % 10));
        scaled /= 10;
    }
    if (negative) segments[0] = 0x40;
    segments[POWER_DECIMAL_SLOT] |= 0x80;
    return true;
}

static inline unsigned power_view_for_press(unsigned current, uint8_t pressed) {
    if (pressed & POWER_KEY_OK_MASK) return POWER_WATTS;
    if (pressed & POWER_KEY_HOME_MASK) return POWER_CURRENT;
    if (pressed & POWER_KEY_START_MASK) return POWER_VOLTAGE;
    return current;
}

static inline const char *power_key_name(uint8_t pressed) {
    if (pressed & POWER_KEY_OK_MASK) return "OK";
    if (pressed & POWER_KEY_HOME_MASK) return "HOME";
    if (pressed & POWER_KEY_START_MASK) return "START";
    return "NONE";
}

typedef struct { bool enabled; unsigned view; } power_control_t;
static inline void power_control_set(power_control_t *c, bool on) {
    c->enabled = on;
    if (on) c->view = POWER_VOLTAGE;
}
static inline void power_control_keys(power_control_t *c, uint8_t pressed) {
    if (c->enabled) c->view = power_view_for_press(c->view, pressed);
}
#endif
