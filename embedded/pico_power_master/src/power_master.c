#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "host_control.h"
#include "bridge_config.h"
#include "power_relay.h"
#include "ina226.h"
#include "panel_link.h"
#include "ws2812.pio.h"

enum { RX_SIZE = 2048 };
static volatile uint8_t rx[RX_SIZE];
static volatile uint32_t head, tail, dropped;
static power_panel_t panel;
static panel_line_t target_line;
static ina226_t sensor;
static power_measurement_t measured;
static bool sensor_valid, reset_active;
static power_control_t control;
static cart_host_t host;
static uint32_t sample_at, led_rgb;
static uint32_t reset_started;
static PIO const led_pio = pio0;
static uint led_sm;
static uint led_dma;
static uint32_t led_buffer[POWER_WS2812_COUNT];

static void on_rx(void) {
    while (uart_is_readable(BRIDGE_UART_ID)) {
        uint8_t value = (uint8_t)uart_getc(BRIDGE_UART_ID);
        if ((uint32_t)(head - tail) < RX_SIZE) { rx[head & (RX_SIZE - 1)] = value; ++head; }
        else ++dropped;
    }
}

static char mode_name(void) { return control.view == POWER_CURRENT ? 'I' : control.view == POWER_WATTS ? 'P' : 'V'; }

static void update_display(void) {
    panel.enabled = control.enabled;
    if (!control.enabled) memset(panel.segments, 0, 4);
    else if (!sensor_valid) power_error_segments(panel.segments);
    else if (control.view == POWER_CURRENT) power_format(measured.current_ua, panel.segments);
    else if (control.view == POWER_WATTS) power_format(measured.power_uw, panel.segments);
    else power_format(measured.bus_uv, panel.segments);
}

static void set_power(bool on, uint32_t now) {
    if (on && !cart_can_power_on(&host, now)) { printf("POWER blocked=estop\n"); return; }
    power_control_set(&control, on);
    gpio_put(POWER_RELAY_PIN, control.enabled ? 1 : 0);
    update_display();
    power_panel_beep(&panel, on ? 100 : 60, now);
    printf("POWER %s relay=%u mode=%c\n", control.enabled ? "ON" : "OFF", control.enabled, mode_name());
}

static void print_status(void) {
    uint32_t now=to_ms_since_boot(get_absolute_time());
    if (sensor_valid)
        printf("online=%u power=%u relay=%u ina=1 mode=%c mv=%lu ua=%lld mw=%lld sw=%u dropped=%lu",
               panel.online, control.enabled, gpio_get(POWER_RELAY_PIN), mode_name(), (unsigned long)(measured.bus_uv / 1000),
               (long long)measured.current_ua, (long long)(measured.power_uw / 1000),
               gpio_get(POWER_BUTTON_PIN), (unsigned long)dropped);
    else printf("online=%u power=%u relay=%u ina=0 mode=%c mv=NA ua=NA mw=NA sw=%u dropped=%lu",
                panel.online, control.enabled, gpio_get(POWER_RELAY_PIN), mode_name(), gpio_get(POWER_BUTTON_PIN),
                (unsigned long)dropped);
    printf(" power_protocol=1 session=%lu seq=%lu uptime_ms=%lu estop=%u host_state=%s stop_reason=%s "
           "sample_age_ms=%lu current_valid=%u shunt_raw=%d led_rgb=%lu keys=%u\n",
           (unsigned long)host.session,(unsigned long)host.sequence,(unsigned long)now,host.estop,
           cart_state_names[host.state],host.stop_reason ? host.stop_reason : "none",
           (unsigned long)(now-sample_at),sensor_valid && measured.shunt_raw!=INT16_MIN && measured.shunt_raw!=INT16_MAX,
           measured.shunt_raw,(unsigned long)led_rgb,panel.keys);
}

static void scan_sensor(void) {
    printf("INA scan SDA26=%u SCL27=%u\n", gpio_get(POWER_I2C_SDA_PIN), gpio_get(POWER_I2C_SCL_PIN));
    unsigned found = 0;
    for (uint8_t address = 0x40; address <= 0x4f; ++address) {
        uint16_t config, manufacturer = 0, die = 0;
        if (!ina226_read(address, 0, &config)) continue;
        bool manufacturer_ok = ina226_read(address, 0xfe, &manufacturer);
        bool die_ok = ina226_read(address, 0xff, &die);
        printf("I2C 0x%02X config=%04X manufacturer=%04X/%u die=%04X/%u\n",
               address, config, manufacturer, manufacturer_ok, die, die_ok);
        ++found;
    }
    if (!found) printf("INA no reply at 0x40..0x4F\n");
}

static void print_panel(void) {
    bool synced = panel.online && panel.enabled_known && panel.segments_known &&
        panel.enabled == panel.acknowledged_enabled &&
        !memcmp(panel.segments, panel.acknowledged_segments, 4);
    printf("PANEL online=%u sync=%u enabled=%u keys=%u seg=%02X,%02X,%02X,%02X beep=%u\n",
           panel.online, synced, panel.acknowledged_enabled, panel.keys,
           panel.acknowledged_segments[0], panel.acknowledged_segments[1],
           panel.acknowledged_segments[2], panel.acknowledged_segments[3],
           POWER_BUZZER_ENABLE && panel.beep_supported);
}

static void start_esp_reset(void) {
    if (reset_active) return;
    gpio_put(BRIDGE_ESP_RESET_PIN, 0);
    gpio_set_dir(BRIDGE_ESP_RESET_PIN, GPIO_OUT);
    gpio_set_dir(BRIDGE_ESP_BOOT_PIN, GPIO_IN);
    uint irq = BRIDGE_UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_enabled(irq, false);
    on_rx(); tail = head;
    irq_set_enabled(irq, true);
    memset(&target_line, 0, sizeof(target_line));
    power_panel_disconnected(&panel, to_ms_since_boot(get_absolute_time()));
    reset_started = time_us_32(); reset_active = true;
    printf("ESP32 reset requested\n");
}

static void console(char *line, uint32_t now) {
    if (!strcmp(line, "cap")) {
        printf("power_protocol=1 watchdog_ms=750 remote_on=0 led_state=1 estop_latch=1 shunt_uohm=%u\n",POWER_SHUNT_UOHM);
    } else if (!strcmp(line,"estop")) {
        cart_stop(&host,"controller"); set_power(false,now); printf("OK estop\n"); print_status();
    } else if (!strncmp(line,"claim ",6)) {
        uint32_t id;
        if(cart_u32(line+6,&id) && cart_claim(&host,id,now)) {
            printf("OK claim=%lu\n",(unsigned long)id); print_status();
        } else printf("ERR claim\n");
    } else if (!strncmp(line,"host ",5)) {
        char sid[12],seq[12],state[16],extra; uint32_t id,n;
        if(sscanf(line+5,"%11s %11s %15s %c",sid,seq,state,&extra)==3 &&
           cart_u32(sid,&id) && cart_u32(seq,&n) && cart_heartbeat(&host,id,n,state,now))
            printf("OK host=%lu seq=%lu\n",(unsigned long)id,(unsigned long)n);
        else printf("ERR host\n");
    } else if (!strncmp(line,"clear ",6)) {
        uint32_t id;
        if(cart_u32(line+6,&id) && cart_clear(&host,id,now,!gpio_get(POWER_BUTTON_PIN),control.enabled)) {
            printf("OK clear=%lu\n",(unsigned long)id); print_status();
        } else printf("ERR clear\n");
    } else if (!strncmp(line,"release ",8)) {
        uint32_t id;
        if(cart_u32(line+8,&id) && host.managed && id==host.session) {
            set_power(false,now); host.managed=false; host.session=0;
            printf("OK release=%lu\n",(unsigned long)id); print_status();
        } else printf("ERR release\n");
    } else if (!strcmp(line, "status")) print_status();
    else if (!strcmp(line, "panel")) print_panel();
    else if (!strcmp(line, "buttons")) {
        printf("BUTTONS ON/OFF=power START=V HOME=I OK=P masks=%u,%u,%u\n",
               POWER_KEY_START_MASK, POWER_KEY_HOME_MASK, POWER_KEY_OK_MASK);
    }
    else if (!strcmp(line, "scan")) scan_sensor();
    else if (!strcmp(line, "off")) set_power(false, now);
    else if (!strcmp(line, "reset")) start_esp_reset();
    else if (!strcmp(line, "beep")) {
        power_panel_beep(&panel, 150, now);
        printf(POWER_BUZZER_ENABLE ? "BEEP requested\n" : "BEEP muted\n");
    }
    else if (!strcmp(line, "mode V") || !strcmp(line, "mode I") || !strcmp(line, "mode P")) {
        control.view = line[5] == 'I' ? POWER_CURRENT : line[5] == 'P' ? POWER_WATTS : POWER_VOLTAGE;
        update_display(); printf("MODE %c\n", mode_name());
    } else printf("Commands: cap | claim ID | host ID SEQ STATE | estop | clear ID | release ID | status | panel | buttons | scan | mode V/I/P | off | reset | beep | help\n");
}

static void init_gpio(void) {
    power_relay_init_off();
    gpio_init(POWER_BUTTON_PIN); gpio_set_dir(POWER_BUTTON_PIN, GPIO_IN); gpio_pull_up(POWER_BUTTON_PIN);
    gpio_init(BRIDGE_ESP_BOOT_PIN); gpio_init(BRIDGE_ESP_RESET_PIN);
    gpio_disable_pulls(BRIDGE_ESP_BOOT_PIN); gpio_disable_pulls(BRIDGE_ESP_RESET_PIN);
    gpio_put(BRIDGE_ESP_BOOT_PIN, 0); gpio_put(BRIDGE_ESP_RESET_PIN, 0);
    gpio_set_dir(BRIDGE_ESP_BOOT_PIN, GPIO_IN); gpio_set_dir(BRIDGE_ESP_RESET_PIN, GPIO_IN);
}

static void init_peripherals(void) {
    uart_init(BRIDGE_UART_ID, BRIDGE_DEFAULT_BAUD);
    gpio_set_function(BRIDGE_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(BRIDGE_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_disable_pulls(BRIDGE_UART_TX_PIN); gpio_disable_pulls(BRIDGE_UART_RX_PIN);
    uart_set_hw_flow(BRIDGE_UART_ID, false, false);
    uart_set_format(BRIDGE_UART_ID, 8, 1, UART_PARITY_NONE);
    uint irq = BRIDGE_UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(irq, on_rx); irq_set_enabled(irq, true);
    uart_set_irq_enables(BRIDGE_UART_ID, true, false);
    i2c_init(POWER_I2C_ID, 100000);
    gpio_set_function(POWER_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(POWER_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(POWER_I2C_SDA_PIN); gpio_pull_up(POWER_I2C_SCL_PIN);
    led_sm = (uint)pio_claim_unused_sm(led_pio, true);
    uint offset = pio_add_program(led_pio, &ws2812_program);
    ws2812_program_init(led_pio, led_sm, offset, POWER_WS2812_PIN, 800000, false);
    led_dma = (uint)dma_claim_unused_channel(true);
    dma_channel_config dma = dma_channel_get_default_config(led_dma);
    channel_config_set_transfer_data_size(&dma, DMA_SIZE_32);
    channel_config_set_read_increment(&dma, true);
    channel_config_set_write_increment(&dma, false);
    channel_config_set_dreq(&dma, pio_get_dreq(led_pio, led_sm, true));
    dma_channel_configure(led_dma, &dma, &led_pio->txf[led_sm], NULL, 0, false);
    // The PIO stalls low before the first frame, establishing a reset interval.
    sleep_us(POWER_WS2812_RESET_US);
}

static void update_led(void) {
    static uint32_t previous = UINT32_MAX;
    static uint32_t sent_at;
    uint8_t red = 0, green = 0, blue = 8;
    if (control.enabled) {
        if (!sensor_valid) { red = 12; blue = 0; }
        else if (!panel.online) { red = 8; blue = 8; }
        else if (control.view == POWER_CURRENT) { green = 8; blue = 12; }
        else if (control.view == POWER_WATTS) { red = 12; green = 6; blue = 0; }
        else { green = 12; blue = 0; }
    }
    if(host.estop) { red=12; green=0; blue=0; }
    else if(host.managed) {
        red=green=blue=0;
        if(host.state==CART_FAULT) red=12;
        else if(!sensor_valid) { red=12; green=4; }
        else if(!panel.online) { red=8; blue=8; }
        else if(host.state==CART_DRIVING) green=12;
        else if(host.state==CART_READY) { green=8; blue=8; }
        else if(host.state==CART_RECOVERING) { red=12; green=6; }
        else if(host.state==CART_STOPPED) red=12;
        else blue=8;
    }
    led_rgb=((uint32_t)red<<16)|((uint32_t)green<<8)|blue;
    uint32_t grb = ((uint32_t)green << 16) | ((uint32_t)red << 8) | blue;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (grb != previous || (uint32_t)(now - sent_at) >= POWER_WS2812_REFRESH_MS) {
        for (unsigned i = 0; i < POWER_WS2812_COUNT; ++i)
            led_buffer[i] = grb << 8;
        // DREQ-paced DMA feeds all pixels even while USB/UART IRQs run.
        dma_channel_transfer_from_buffer_now(led_dma, led_buffer, POWER_WS2812_COUNT);
        dma_channel_wait_for_finish_blocking(led_dma);
        // DMA completion only means the FIFO was filled, not that DIN is idle.
        while (!pio_sm_is_tx_fifo_empty(led_pio, led_sm)) tight_loop_contents();
        // At most one 24-bit pixel (30 us at 800 kHz) is still in the shifter.
        sleep_us(30u + POWER_WS2812_RESET_US);
        previous = grb;
        sent_at = to_ms_since_boot(get_absolute_time());
    }
}

int main(void) {
    init_gpio(); // Relay latch is OFF before enabling its output driver.
    init_peripherals();
    stdio_init_all(); stdio_set_translate_crlf(&stdio_usb, false);
    power_panel_init(&panel);
    power_button_t button = {0};
    panel_line_t host_line = {0};
    uint32_t status_at=0;
    uint32_t sampled = 0, displayed = 0, probed = (uint32_t)-2000, button_at = 0, last_overflow = 0;
    bool was_online = false;
    printf("RP2040 power master: relay GP2 High=ON; button GP5 active Low; INA226 SDA26/SCL27 R100\n");
    watchdog_enable(2000, true);
    while (true) {
        watchdog_update();
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (cart_watchdog(&host,now,stdio_usb_connected())) set_power(false,now);
        if (reset_active && (uint32_t)(time_us_32() - reset_started) >= BRIDGE_ESP_RESET_HOLD_US) {
            gpio_set_dir(BRIDGE_ESP_RESET_PIN, GPIO_IN);
            reset_active = false; power_panel_disconnected(&panel, now);
        }
        if ((uint32_t)(now - button_at) >= 5) {
            button_at = now;
            if (power_button_sample(&button, !gpio_get(POWER_BUTTON_PIN), now)) set_power(!control.enabled, now);
        }
        if (!sensor.found && (uint32_t)(now - probed) >= 2000) {
            probed = sampled = now;
            if (ina226_find(&sensor))
                printf("INA226 found address=0x%02X die=0x%04X shunt_uohm=%u\n",
                       sensor.address, sensor.die_id, POWER_SHUNT_UOHM);
        }
        if (sensor.found && (uint32_t)(now - sampled) >= POWER_SAMPLE_MS) {
            sampled = now;
            sensor_valid = ina226_sample(&sensor, &measured);
            if(sensor_valid) sample_at=now;
            if (!sensor_valid) { sensor.found = false; probed = now; printf("INA226 read failed\n"); }
        }
        if (last_overflow != dropped) { target_line.discard = true; last_overflow = dropped; }
        while (tail != head) {
            uint8_t value = rx[tail & (RX_SIZE - 1)]; ++tail;
            if (panel_line_feed(&target_line, value)) {
                panel_reply_t reply;
                if (panel_parse_reply(target_line.text, &reply)) {
                    uint8_t pressed = power_panel_reply(&panel, &reply, now);
                    if (pressed && control.enabled) {
                        power_control_keys(&control, pressed);
                        update_display(); power_panel_beep(&panel, 40, now);
                        printf("KEY pressed=%u name=%s MODE %c\n", pressed, power_key_name(pressed), mode_name());
                    } else if (pressed) printf("KEY pressed=%u name=%s ignored=off\n", pressed, power_key_name(pressed));
                }
            }
        }
        for (unsigned i = 0; i < 64; ++i) {
            int c = getchar_timeout_us(0);
            if (c == PICO_ERROR_TIMEOUT) break;
            if (panel_line_feed(&host_line, (unsigned char)c)) console(host_line.text, now);
        }
        if ((uint32_t)(now - displayed) >= POWER_DISPLAY_MS) { displayed = now; update_display(); }
        char request[PANEL_LINE_SIZE];
        if (!reset_active && power_panel_tick(&panel, now, request, sizeof(request)))
            uart_write_blocking(BRIDGE_UART_ID, (const uint8_t *)request, strlen(request));
        if (was_online != panel.online) {
            printf("ESP32 %s; GP%u TX / GP%u RX, 115200 baud\n",
                   panel.online ? "online" : "offline", BRIDGE_UART_TX_PIN, BRIDGE_UART_RX_PIN);
            was_online = panel.online;
        }
        update_led();
        if(host.managed && (uint32_t)(now-status_at)>=100) { status_at=now; print_status(); }
        sleep_ms(1);
    }
}
