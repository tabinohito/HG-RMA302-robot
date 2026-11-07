#include <Arduino.h>
#include <SPI.h>
#include "panel_test.h"

// ESP32 GPIO numbers, not Pico GPIO numbers. See docs/FIRMWARE_FINDINGS.md.
static constexpr uint8_t DATA = 25, CLOCK = 33, SELECT = 32, OE = 26, WAKE = 2;
static constexpr uint8_t BUZZER = 27;
// Protocol key bits 0/1/2 match the original firmware's masks 1/2/4.
static constexpr uint8_t KEY_PINS[] = {22, 21, 19};
static SPIClass display_spi(HSPI);
static panel_state_t panel;
static panel_debounce_t debounce;
static panel_line_t input;
static uint32_t refreshed_at, sampled_at;
static unsigned slot;
static uint32_t buzz_started, buzz_ms;

static void send_word(uint16_t word) {
    // Stock: SPI mode 0, 400kHz, GPIO32 CS, high byte first. GPIO2 is wake,
    // not a per-word latch. OE is active low in the verified initialization.
    digitalWrite(OE, HIGH);
    display_spi.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
    digitalWrite(SELECT, LOW);
    display_spi.transfer((uint8_t)(word >> 8));
    display_spi.transfer((uint8_t)word);
    digitalWrite(SELECT, HIGH);
    display_spi.endTransaction();
    digitalWrite(OE, panel.enabled ? LOW : HIGH);
}

void setup() {
    digitalWrite(OE, HIGH); pinMode(OE, OUTPUT);
    digitalWrite(WAKE, HIGH); pinMode(WAKE, OUTPUT);
    digitalWrite(SELECT, HIGH); pinMode(SELECT, OUTPUT);
    digitalWrite(BUZZER, LOW); pinMode(BUZZER, OUTPUT);
    for (uint8_t pin : KEY_PINS) pinMode(pin, INPUT_PULLUP);
    panel_state_init(&panel);
#ifdef PANEL_SLAVE_BOOT_BLANK
    for (unsigned i = 0; i < 4; ++i) panel.words[i] = panel_word(i, 0);
#endif
    display_spi.begin(CLOCK, -1, DATA, SELECT);
    display_spi.setHwCs(false);
    Serial.setRxBufferSize(512);
    Serial.begin(115200, SERIAL_8N1, 3, 1); // Existing board R/T programming header.
    Serial.setDebugOutput(false);
    // A leading newline separates the protocol from ROM/bootloader output.
    Serial.print("\nP1 0 READY 1\n");
}

void loop() {
    uint32_t now = millis();
    if (buzz_ms && (uint32_t)(now - buzz_started) >= buzz_ms) {
        digitalWrite(BUZZER, LOW);
        buzz_ms = 0;
    }
    if ((uint32_t)(now - refreshed_at) >= 2) {
        refreshed_at = now;
        send_word(panel.words[slot]);
        slot = (slot + 1) % 4;
    }
    if ((uint32_t)(now - sampled_at) >= 5) {
        sampled_at = now;
        uint8_t raw = 0;
        for (unsigned i = 0; i < PANEL_KEY_COUNT; ++i)
            if (digitalRead(KEY_PINS[i]) == LOW) raw |= (uint8_t)(1u << i);
        uint8_t changed = panel_debounce(&debounce, raw);
        if (changed) {
            panel.keys = debounce.stable;
            ++panel.event_id;
            Serial.printf("P1 0 KEY %lu %u %u\n", (unsigned long)panel.event_id, panel.keys, changed);
        }
    }
    // Bound one turn so a continuous UART stream cannot starve display refresh.
    for (unsigned i = 0; i < 64 && Serial.available(); ++i) {
        int c = Serial.read();
        if (c < 0 || !panel_line_feed(&input, (unsigned char)c)) continue;
        panel_request_t request = {};
        char reply[PANEL_LINE_SIZE];
        if (panel_parse_request(input.text, &request)) {
            panel_handle(&panel, &request, reply, sizeof(reply));
            if (request.op == PANEL_BEEP) {
                buzz_started = now;
                buzz_ms = request.beep_ms;
                digitalWrite(BUZZER, buzz_ms ? HIGH : LOW);
            }
            Serial.print(reply);
        } else if (request.id) Serial.printf("P1 %u ERR ARG\n", request.id);
    }
    delay(1);
}
