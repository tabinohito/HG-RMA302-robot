"""Run the production RP2040 main loop against simulated pins, INA226 and ESP32.

Only hardware/SDK boundaries are replaced. The real UART protocol, debouncer,
sensor driver, formatter, relay logic and recovery loop execute together.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

from sdk_mock import SDK
from test_power import I2C_SDK

ROOT = Path(__file__).resolve().parents[1] / "pico_power_master"

SDK_INTEGRATED = SDK.replace(
    "static void uart_write_blocking(uart_inst_t *u, const uint8_t *b, uint32_t n) {\n"
    "    for (uint32_t i = 0; i < n; ++i) uart_output[uart_out_size++] = b[i];\n}",
    "static void uart_write_blocking(uart_inst_t *u, const uint8_t *b, uint32_t n);"
).replace(
    "static void sleep_ms(uint32_t delay) { now_us += delay * 1000; }",
    "static void sleep_ms(uint32_t delay);"
).replace(
    "static void tight_loop_contents(void) {}",
    "static void advance_us(uint32_t delay);\n"
    "static void tight_loop_contents(void) { advance_us(1); }"
).replace(
    "static bool gpio_get(uint p) { return !(gpio_outputs & (1u << p)) || (gpio_high & (1u << p)); }",
    "static bool switch_down;\n"
    "static bool gpio_get(uint p) { return p == 5 ? !switch_down : "
    "(!(gpio_outputs & (1u << p)) || (gpio_high & (1u << p))); }"
) + r'''
#define GPIO_FUNC_I2C 3
static bool mock_usb_connected = true;
static bool stdio_usb_connected(void) { return mock_usb_connected; }
static void watchdog_enable(uint32_t ms, bool pause) { if(ms != 2000 || !pause) __builtin_trap(); }
static void watchdog_update(void) {}
static void gpio_pull_up(uint pin) {}
static void i2c_init(void *bus, uint32_t baud) { if (baud != 100000) __builtin_trap(); }
typedef struct { uint32_t txf[4]; } pio_hw_t;
typedef pio_hw_t *PIO;
static pio_hw_t mock_pio;
#define pio0 (&mock_pio)
// Model the user's 30-pixel chain independently of the firmware count setting.
static uint32_t led_pixels[30], led_frame[30];
static unsigned led_pending, led_changes;
static uint32_t led_last_bit_us, led_dma_done_us;
static bool led_dma_active;
static int ws2812_program;
static int pio_claim_unused_sm(PIO p, bool required) { return 0; }
static uint pio_add_program(PIO p, const void *program) { return 0; }
static void ws2812_program_init(PIO p, uint sm, uint offset, uint pin, float hz, bool rgbw) {
    if (pin != 28 || hz != 800000 || rgbw) __builtin_trap();
}
static uint pio_get_dreq(PIO p, uint sm, bool tx) {
    if (p != pio0 || sm != 0 || !tx) __builtin_trap();
    return 7;
}
static bool pio_sm_is_tx_fifo_empty(PIO p, uint sm) {
    // FIFO empties while the final pixel is still being shifted out.
    return now_us >= led_last_bit_us - 30;
}
#define DMA_SIZE_32 2
typedef struct { uint size, dreq; bool read_inc, write_inc; } dma_channel_config;
static int dma_claim_unused_channel(bool required) { return 2; }
static dma_channel_config dma_channel_get_default_config(uint ch) {
    return (dma_channel_config){0};
}
static void channel_config_set_transfer_data_size(dma_channel_config *c, uint size) { c->size = size; }
static void channel_config_set_read_increment(dma_channel_config *c, bool inc) { c->read_inc = inc; }
static void channel_config_set_write_increment(dma_channel_config *c, bool inc) { c->write_inc = inc; }
static void channel_config_set_dreq(dma_channel_config *c, uint req) { c->dreq = req; }
static void dma_channel_configure(uint ch, const dma_channel_config *c,
                                  volatile void *to, const void *from, uint count, bool start) {
    if (ch != 2 || c->size != DMA_SIZE_32 || c->dreq != 7 || !c->read_inc || c->write_inc ||
        to != &pio0->txf[0] || from || count || start) __builtin_trap();
}
static void dma_channel_transfer_from_buffer_now(uint ch, const void *from, uint count) {
    // A partial frame or too-short LOW reset interval must never be accepted.
    if (ch != 2 || count != 30 || led_dma_active || led_pending ||
        now_us < led_last_bit_us + 300) __builtin_trap();
    for (unsigned i = 0; i < count; ++i) led_frame[i] = ((const uint32_t *)from)[i];
    led_pending = count;
    led_dma_active = true;
    led_last_bit_us = now_us + 30 * count; // 24 bits at 800 kHz.
    led_dma_done_us = now_us + 30 * (count - 9); // Joined FIFO + output shifter.
}
static void dma_channel_wait_for_finish_blocking(uint ch) {
    if (ch != 2 || !led_dma_active || !uart_irq_enabled) __builtin_trap();
    if (now_us < led_dma_done_us) advance_us(led_dma_done_us - now_us);
    led_dma_active = false;
    // Simulate an interrupt delaying the CPU beyond the FIFO's buffered time.
    // DMA keeps the stream continuous while the CPU is unavailable.
    advance_us(TEST_LED_IRQ_US);
}
static void sleep_us(uint64_t delay) { advance_us((uint32_t)delay); }
'''

PROGRAM = r'''
#include <assert.h>
#include <setjmp.h>
#define main firmware_main
#include "src/power_master.c"
#undef main

static panel_state_t slave;
static bool transport_ok = true;
static unsigned buzzes[16], buzz_count;
static jmp_buf finished;

static void expect_buzz(unsigned count, unsigned last_ms) {
    assert(buzz_count == (POWER_BUZZER_ENABLE ? count : 0));
    if (POWER_BUZZER_ENABLE && count) assert(buzzes[count - 1] == last_ms);
}

static void expect_leds(uint32_t value) {
    assert(led_changes);
    for (unsigned i = 0; i < 30; ++i) assert(led_pixels[i] == value);
}

static void incoming_line(const char *text) {
    in_read = 0; in_size = (uint32_t)strlen(text);
    memcpy(incoming, text, in_size);
    on_rx();
}

static void uart_write_blocking(uart_inst_t *u, const uint8_t *b, uint32_t n) {
    assert(u == uart1 && n < PANEL_LINE_SIZE);
    if (!transport_ok) return;
    char line[PANEL_LINE_SIZE], reply[PANEL_LINE_SIZE];
    memcpy(line, b, n); line[n] = 0; line[strcspn(line, "\r\n")] = 0;
    panel_request_t request;
    assert(panel_parse_request(line, &request));
    if (request.op == PANEL_BEEP) {
        assert(buzz_count < 16); buzzes[buzz_count++] = request.beep_ms;
    }
    panel_handle(&slave, &request, reply, sizeof(reply));
    incoming_line(reply);
}

static void keys(uint8_t mask) {
    uint8_t changed = slave.keys ^ mask;
    slave.keys = mask; ++slave.event_id;
    char event[128];
    snprintf(event, sizeof(event), "P1 0 KEY %lu %u %u\n",
             (unsigned long)slave.event_id, mask, changed);
    incoming_line(event);
}

static void displayed(const char *digits, int dot) {
    // dot is the SPI word index, not the visible decimal position: slot 3
    // drives the lower center colon dot between the second and third digits.
    assert(slave.enabled && panel.online && sensor_valid);
    for (unsigned i = 0; i < 4; ++i) {
        uint8_t segments = (uint8_t)panel_glyph(digits[i]);
        if ((int)i == dot) segments |= 0x80;
        assert(slave.words[i] == panel_word(i, segments));
    }
}

static void scenario(uint32_t ms) {
    switch (ms) {
    case 250:
        assert(sensor_valid && sensor.address == 0x44 && measured.bus_uv == 12000000);
        assert(panel.online && !control.enabled && !gpio_get(2) && !slave.enabled);
        expect_leds(0x00000800); assert(buzz_count == 0);
        break;
    case 300: switch_down = true; break;
    case 450:
        assert(control.enabled && gpio_get(2)); displayed("1200", 3);
        expect_leds(0x0c000000); expect_buzz(1, 100);
        break;
    case 500:
        // Model a lost update in the tail. With no mode change, refresh must
        // still restore the whole chain within the 250 ms refresh interval.
        for (unsigned i = 15; i < 30; ++i) led_pixels[i] = 0x00000800;
        break;
    case 600: switch_down = false; break;
    case 690: expect_leds(0x0c000000); break;
    case 700: keys(2); break;
    case 750:
        assert(control.view == POWER_CURRENT && gpio_get(2)); displayed("0010", 3);
        expect_leds(0x08000c00); expect_buzz(2, 40);
        break;
    case 800: keys(0); break;
    case 850: keys(4); break;
    case 950:
        assert(control.view == POWER_WATTS && gpio_get(2)); displayed("0120", 3);
        expect_leds(0x060c0000); expect_buzz(3, 40);
        break;
    case 975: keys(0); break;
    case 1000: mock_present = false; break;
    case 1250:
        assert(!sensor_valid && control.enabled && gpio_get(2));
        for (unsigned i = 0; i < 4; ++i) assert(slave.words[i] == panel_word(i, 0x40));
        expect_leds(0x000c0000);
        break;
    case 1300: mock_present = true; break;
    case 3400:
        assert(sensor_valid && gpio_get(2)); displayed("0120", 3);
        expect_leds(0x060c0000);
        break;
    case 3500: transport_ok = false; break;
    case 4200:
        assert(!panel.online && sensor_valid && control.enabled && gpio_get(2));
        expect_leds(0x00080800);
        transport_ok = true; panel_state_init(&slave);
        for (unsigned i = 0; i < 4; ++i) slave.words[i] = panel_word(i, 0);
        incoming_line("P1 0 READY 1\n");
        break;
    case 4400:
        displayed("0120", 3); expect_buzz(3, 40); // No replay after reconnect.
        break;
    case 4500: keys(1); break;
    case 4600:
        displayed("1200", 3); expect_leds(0x0c000000); expect_buzz(4, 40);
        keys(0);
        break;
    case 4700: switch_down = true; break;
    case 4800:
        assert(!control.enabled && !gpio_get(2) && !slave.enabled);
        expect_leds(0x00000800); expect_buzz(5, 60);
        for (unsigned i = 0; i < 4; ++i) assert(slave.words[i] == panel_word(i, 0));
        break;
    case 4850:
        if (!POWER_BUZZER_ENABLE) {
            char cmd[] = "beep"; console(cmd, now_us / 1000);
        }
        break;
    case 4900: keys(4); break;
    case 4950:
        assert(!control.enabled && !gpio_get(2) && !slave.enabled);
        assert(control.view == POWER_VOLTAGE && dropped == 0); expect_buzz(5, 60);
        assert(led_changes >= 9);
        longjmp(finished, 1);
    }
}

static void advance_us(uint32_t delay) {
    uint32_t until = now_us + delay;
    while (now_us < until) {
        uint32_t boundary = (now_us / 1000 + 1) * 1000;
        now_us = boundary < until ? boundary : until;
        // LEDs latch automatically after the final bit followed by LOW.
        if (led_pending && now_us >= led_last_bit_us + 300) {
            assert(led_pending == 30);
            memcpy(led_pixels, led_frame, sizeof(led_pixels));
            led_pending = 0;
            ++led_changes;
        }
        if (now_us % 1000 == 0) scenario(now_us / 1000);
    }
}

static void sleep_ms(uint32_t delay) {
    // update_led must finish both the trailing pixel and reset, even if no
    // interrupt has lengthened the interval after DMA completion.
    assert(!led_dma_active && !led_pending);
    advance_us(delay * 1000);
}

int main(void) {
    mock_present = true; mock_bus = 9600; mock_shunt = 4000;
    panel_state_init(&slave);
    if (!setjmp(finished)) firmware_main();
    assert(now_us == 4950000);
    return 0;
}
'''


class IntegratedPowerTests(unittest.TestCase):
    def run_controller_flow(self, buzzer, irq_delay_us=500, program=PROGRAM):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            (tmp / "fake_sdk.h").write_text(SDK_INTEGRATED)
            for name in ("pico/stdlib.h", "pico/stdio_usb.h", "hardware/gpio.h",
                         "hardware/irq.h", "hardware/watchdog.h", "hardware/pio.h", "hardware/dma.h", "hardware/uart.h", "ws2812.pio.h"):
                path = tmp / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('#include "fake_sdk.h"\n')
            (tmp / "hardware/i2c.h").write_text(I2C_SDK)
            (tmp / "test.c").write_text(program)
            subprocess.run(["cc", "-std=c11", "-Werror=implicit-function-declaration",
                            f"-DTEST_LED_IRQ_US={irq_delay_us}",
                            *(["-DPOWER_BUZZER_ENABLE=1"] if buzzer else []),
                            "-I", str(tmp), "-I", str(ROOT), "-I", str(ROOT / "panel"),
                            "-I", str(ROOT / "power"), str(tmp / "test.c"),
                            "-o", str(tmp / "test")], check=True)
            subprocess.run([str(tmp / "test")], check=True, timeout=10)

    def test_default_muted_controller_flow_and_recovery(self):
        self.run_controller_flow(buzzer=False)

    def test_explicit_sound_enabled_controller_flow_and_recovery(self):
        self.run_controller_flow(buzzer=True)

    def test_dma_fifo_drain_and_reset_without_cpu_delay(self):
        self.run_controller_flow(buzzer=False, irq_delay_us=0)


if __name__ == "__main__":
    unittest.main()
