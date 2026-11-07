"""Measurement, button and UART recovery checks for the power controller."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1] / "pico_power_master"

MONITOR = r'''
#include <assert.h>
#include <limits.h>
#include "monitor.h"
static void display(int64_t value, const char *digits) {
    uint8_t segments[4]; assert(power_format(value, segments));
    for (unsigned i = 0; i < 4; ++i)
        // Real-board probe: fourth word bit 7 lights the LOWER center dot.
        assert(segments[i] == (panel_glyph(digits[i]) | (i == 3 ? 0x80 : 0)));
}
static void overflow(int64_t value) {
    uint8_t segments[4]; assert(!power_format(value, segments));
    for (unsigned i = 0; i < 4; ++i) assert(segments[i] == 0x40);
}
int main(void) {
    power_measurement_t m = power_measure(9600, 4000, 100000);
    assert(m.bus_uv == 12000000 && m.current_ua == 100000 && m.power_uw == 1200000);
    m = power_measure(9600, (uint16_t)-4000, 100000);
    assert(m.shunt_raw == -4000 && m.current_ua == -100000 && m.power_uw == -1200000);
    m = power_measure(28800, 0x8000, 100000);
    assert(m.bus_uv == 36000000 && m.current_ua == -819200 && m.power_uw == -29491200);
    display(12000000, "1200");
    display(123456, "0012");
    display(-123456, "-012");
    display(61090, "0006"); // 61.09 mA is 00.06 A, not 61.09.
    display(1200000, "0120");
    display(4999, "0000");
    display(-4999, "0000"); // No negative zero.
    display(5000, "0001");
    display(-5000, "-001");
    display(0, "0000");
    display(99994999, "9999");
    display(-9994999, "-999");
    overflow(99995000); // Rounding must not move the decimal point.
    overflow(-9995000);
    overflow(INT64_MIN);
    overflow(INT64_MAX);

    power_button_t b = {0};
    assert(!power_button_sample(&b, true, 0)); // Held at boot must not turn on.
    assert(!power_button_sample(&b, true, 100));
    assert(!power_button_sample(&b, false, 105));
    assert(!power_button_sample(&b, false, 130));
    assert(!power_button_sample(&b, true, 140));
    assert(!power_button_sample(&b, false, 145)); // Bounce.
    assert(!power_button_sample(&b, true, 150));
    assert(!power_button_sample(&b, true, 174));
    assert(power_button_sample(&b, true, 175));
    assert(!power_button_sample(&b, true, 1000));
    assert(!power_button_sample(&b, false, UINT32_MAX - 10));
    assert(!power_button_sample(&b, false, 15)); // Release across rollover.
    assert(!power_button_sample(&b, true, 20));
    assert(power_button_sample(&b, true, 45));

    power_control_t c = {0};
    power_control_keys(&c, 7); assert(!c.enabled && c.view == POWER_VOLTAGE);
    power_control_set(&c, true); power_control_keys(&c, 2);
    assert(c.enabled && c.view == POWER_CURRENT);
    power_control_keys(&c, 4); assert(c.view == POWER_WATTS);
    power_control_keys(&c, 1); assert(c.view == POWER_VOLTAGE);
    power_control_set(&c, false); power_control_keys(&c, 4);
    assert(!c.enabled && c.view == POWER_VOLTAGE);
    power_control_set(&c, true); assert(c.view == POWER_VOLTAGE);
    return 0;
}
'''

LINK = r'''
#include <assert.h>
#include "panel_link.h"
static unsigned receive(power_panel_t *p, const char *text, uint32_t now) {
    char line[128]; strcpy(line, text); panel_reply_t r;
    assert(panel_parse_reply(line, &r)); return power_panel_reply(p, &r, now);
}
static void ack(power_panel_t *p, const char *operation, uint32_t now) {
    char line[128]; snprintf(line, sizeof(line), "P1 %u OK %s", p->pending, operation);
    assert(!receive(p, line, now));
}
int main(void) {
    power_panel_t p; power_panel_init(&p); char out[128];
    assert(power_panel_tick(&p, 0, out, sizeof(out)) && strstr(out, "PING"));
    ack(&p, "PONG", 1); assert(p.online);
    // A standby connection first disables any stale display from the slave.
    assert(power_panel_tick(&p, 2, out, sizeof(out)) && strstr(out, "ENABLE 0"));
    ack(&p, "ENABLE", 3);
    assert(power_panel_tick(&p, 4, out, sizeof(out)) && strstr(out, "SEG 00 00 00 00"));
    p.segments[0] = 0x06; ack(&p, "SEG", 5); // Desired changed in flight.
    assert(power_panel_tick(&p, 6, out, sizeof(out)) && strstr(out, "SEG 06 00 00 00"));
    ack(&p, "SEG", 7);
    p.enabled = true;
    assert(power_panel_tick(&p, 8, out, sizeof(out)) && strstr(out, "ENABLE 1"));
    ack(&p, "ENABLE", 9);
    assert(power_panel_tick(&p, 10, out, sizeof(out)) && strstr(out, "KEYS"));
    char line[128]; snprintf(line, sizeof(line), "P1 %u OK KEYS 1 10", p.pending);
    assert(!receive(&p, line, 11)); // Held on first connection is a baseline.
    assert(!receive(&p, "P1 0 KEY 11 0 1", 12));
    assert(receive(&p, "P1 0 KEY 12 2 2", 13) == 2);
    assert(!receive(&p, "P1 0 KEY 12 2 2", 14)); // Duplicate.
    assert(!receive(&p, "P1 0 KEY 11 0 2", 15)); // Stale poll/event.
    assert(p.keys == 2);
    power_panel_beep(&p, 100, 16);
    assert(power_panel_tick(&p, 17, out, sizeof(out)) && strstr(out, "BEEP 100"));
    assert(!p.beep_ms);
    assert(power_panel_tick(&p, 517, out, sizeof(out)) && strstr(out, "PING"));
    assert(!p.online && !p.beep_ms); // Lost ACK must not repeat a beep.
    ack(&p, "PONG", 518);
    p.enabled_known = p.segments_known = true; p.acknowledged_enabled = true;
    memcpy(p.acknowledged_segments, p.segments, 4); p.keys_known = true; p.polled_at = 519;
    power_panel_beep(&p, 150, 519);
    assert(power_panel_tick(&p, 520, out, sizeof(out)) && strstr(out, "BEEP 150"));
    snprintf(line, sizeof(line), "P1 %u ERR ARG", p.pending);
    receive(&p, line, 521); assert(p.online && !p.beep_supported && !p.pending);
    power_panel_beep(&p, 150, 522); assert(!p.beep_ms); // Older slave fallback.
    receive(&p, "P1 0 READY 1", 523);
    assert(!p.online && !p.keys_known && p.beep_supported);
    assert(power_panel_tick(&p, 523, out, sizeof(out)) && strstr(out, "PING"));

    // The slave accepts bounded pulses and cancellation without altering display.
    panel_state_t state; panel_state_init(&state); uint16_t before[4];
    memcpy(before, state.words, sizeof(before));
    panel_request_t r; strcpy(line, "P1 25 BEEP 2000");
    assert(panel_parse_request(line, &r) && r.op == PANEL_BEEP && r.beep_ms == 2000);
    panel_handle(&state, &r, out, sizeof(out)); assert(strstr(out, "OK BEEP"));
    assert(!memcmp(before, state.words, sizeof(before)));
    strcpy(line, "P1 25 BEEP 2001"); assert(!panel_parse_request(line, &r));
    strcpy(line, "P1 25 BEEP 0"); assert(panel_parse_request(line, &r) && r.beep_ms == 0);
    return 0;
}
'''

I2C = r'''
#include <assert.h>
#include "ina226.h"
int main(void) {
    ina226_t sensor = {0}; power_measurement_t m;
    assert(!ina226_sample(&sensor, &m));
    mock_present = false; assert(!ina226_find(&sensor));
    mock_present = true; mock_manufacturer = 0; assert(!ina226_find(&sensor));
    mock_manufacturer = 0x5449; mock_die = 0x1234; assert(!ina226_find(&sensor));
    mock_die = 0x2261; assert(ina226_find(&sensor));
    assert(sensor.found && sensor.address == 0x44 && sensor.die_id == 0x2261);
    assert(mock_config == 0x4527);
    mock_bus = 9600; mock_shunt = (uint16_t)-4000;
    assert(ina226_sample(&sensor, &m));
    assert(m.bus_uv == 12000000 && m.current_ua == -100000 && m.power_uw == -1200000);
    mock_short = true; assert(!ina226_sample(&sensor, &m));
    mock_short = false; mock_bus = 0x8000; assert(!ina226_sample(&sensor, &m));
    mock_present = false; assert(!ina226_sample(&sensor, &m));
    return 0;
}
'''

I2C_SDK = r'''
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#define i2c1 ((void *)2)
static bool mock_present, mock_short;
static uint16_t mock_manufacturer = 0x5449, mock_die = 0x2260;
static uint16_t mock_config, mock_bus, mock_shunt;
static uint8_t mock_register;
static int i2c_write_timeout_us(void *bus, uint8_t addr, const uint8_t *data,
                              size_t size, bool nostop, unsigned timeout) {
    assert(bus == i2c1 && timeout == 2000);
    if (!mock_present || addr != 0x44) return -1;
    if (size == 1) { assert(nostop); mock_register = data[0]; }
    else { assert(size == 3 && !nostop && data[0] == 0); mock_config = data[1] << 8 | data[2]; }
    return (int)size;
}
static int i2c_read_timeout_us(void *bus, uint8_t addr, uint8_t *data,
                             size_t size, bool nostop, unsigned timeout) {
    assert(bus == i2c1 && addr == 0x44 && size == 2 && !nostop && timeout == 2000);
    if (mock_short) return 1;
    uint16_t value = mock_register == 0xfe ? mock_manufacturer : mock_register == 0xff ? mock_die :
                     mock_register == 2 ? mock_bus : mock_shunt;
    data[0] = value >> 8; data[1] = value & 255; return 2;
}
'''


class PowerTests(unittest.TestCase):
    def compile_run(self, program, sdk=None, buzzer=False):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            if sdk:
                (tmp / "hardware").mkdir()
                (tmp / "hardware/i2c.h").write_text(sdk)
            (tmp / "test.c").write_text(program)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-DPOWER_BUZZER_ENABLE=" + str(int(buzzer)),
                            "-I", str(tmp), "-I", str(ROOT / "panel"),
                            "-I", str(ROOT / "power"), "-I", str(ROOT / "src"),
                            str(tmp / "test.c"), "-o", str(tmp / "test")], check=True)
            subprocess.run([str(tmp / "test")], check=True)

    def test_r100_measurement_display_and_physical_power_button(self):
        self.compile_run(MONITOR)

    def test_panel_recovery_inflight_updates_keys_and_buzzer(self):
        self.compile_run(LINK, buzzer=True)

    def test_ina226_identity_endianness_config_and_bus_errors(self):
        self.compile_run(I2C, I2C_SDK)


if __name__ == "__main__":
    unittest.main()
