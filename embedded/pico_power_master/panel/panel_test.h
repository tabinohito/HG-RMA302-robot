#ifndef PANEL_TEST_H
#define PANEL_TEST_H

// Small, bounded ASCII protocol shared by the ESP32 test server and Pico client.
// P1 <id> PING | KEYS | SHOW 1234 | SEG 3F 06 5B 4F | RAW 203F 1006 085B 044F
// P1 <id> BLANK | ENABLE 0/1 | BEEP <0..2000ms>. ID 0 is for READY/KEY events.
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

enum { PANEL_LINE_SIZE = 128, PANEL_KEY_COUNT = 3 };
enum { PANEL_PING, PANEL_KEYS, PANEL_SHOW, PANEL_SEG, PANEL_RAW,
       PANEL_BLANK, PANEL_ENABLE, PANEL_BEEP };
typedef struct {
    char text[PANEL_LINE_SIZE];
    size_t used;
    bool discard;
} panel_line_t;

// Reject an entire overlong/binary line; never interpret a truncated command.
static inline bool panel_line_feed(panel_line_t *line, unsigned char c) {
    if (c == '\n') {
        bool ready = !line->discard && line->used != 0;
        line->text[line->used] = 0;
        line->used = 0;
        line->discard = false;
        return ready;
    }
    if (c == '\r') return false;
    if (c < 32 || c > 126 || line->used + 1 >= sizeof(line->text)) {
        line->discard = true;
    }
    if (!line->discard) line->text[line->used++] = (char)c;
    return false;
}

static inline bool panel_number(const char *s, unsigned base, uint32_t max, uint32_t *out) {
    uint32_t value = 0;
    if (!*s) return false;
    for (; *s; ++s) {
        unsigned digit;
        if (*s >= '0' && *s <= '9') digit = (unsigned)(*s - '0');
        else if (*s >= 'a' && *s <= 'f') digit = (unsigned)(*s - 'a') + 10;
        else if (*s >= 'A' && *s <= 'F') digit = (unsigned)(*s - 'A') + 10;
        else return false;
        if (digit >= base || digit > max || value > (max - digit) / base) return false;
        value = value * base + digit;
    }
    *out = value;
    return true;
}

static inline int panel_tokens(char *s, char **tokens, int limit) {
    int count = 0;
    while (*s) {
        while (*s == ' ') ++s;
        if (!*s) break;
        if (count == limit) return -1;
        tokens[count++] = s;
        while (*s && *s != ' ') ++s;
        if (*s) *s++ = 0;
    }
    return count;
}

static inline int panel_glyph(char c) {
    static const uint8_t digits[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
    if (c >= '0' && c <= '9') return digits[c - '0'];
    if (c == '-') return 0x40;
    if (c == '_') return 0;
    return -1;
}

static inline uint16_t panel_word(unsigned slot, uint8_t segments) {
    // Verified stock function 0x400dff18: bits 13,12,11,10, one per slot.
    return slot < 4 ? (uint16_t)((0x2000u >> slot) | segments) : 0;
}

typedef struct {
    uint16_t id;
    unsigned op;
    uint16_t words[4];
    unsigned enabled;
    unsigned beep_ms;
} panel_request_t;

static inline bool panel_parse_request(char *line, panel_request_t *request) {
    char *t[8];
    uint32_t id, value;
    request->id = 0;
    int n = panel_tokens(line, t, 8);
    if (n < 3 || strcmp(t[0], "P1") || !panel_number(t[1], 10, 65535, &id) || !id) return false;
    request->id = (uint16_t)id;
    if (!strcmp(t[2], "PING") && n == 3) request->op = PANEL_PING;
    else if (!strcmp(t[2], "KEYS") && n == 3) request->op = PANEL_KEYS;
    else if (!strcmp(t[2], "BLANK") && n == 3) request->op = PANEL_BLANK;
    else if (!strcmp(t[2], "ENABLE") && n == 4 && panel_number(t[3], 10, 1, &value)) {
        request->op = PANEL_ENABLE;
        request->enabled = (unsigned)value;
    } else if (!strcmp(t[2], "BEEP") && n == 4 && panel_number(t[3], 10, 2000, &value)) {
        request->op = PANEL_BEEP;
        request->beep_ms = (unsigned)value;
    } else if (!strcmp(t[2], "SHOW") && n == 4 && strlen(t[3]) == 4) {
        request->op = PANEL_SHOW;
        for (unsigned i = 0; i < 4; ++i) {
            int segments = panel_glyph(t[3][i]);
            if (segments < 0) return false;
            request->words[i] = panel_word(i, (uint8_t)segments);
        }
    } else if (n == 7 && (!strcmp(t[2], "SEG") || !strcmp(t[2], "RAW"))) {
        request->op = !strcmp(t[2], "SEG") ? PANEL_SEG : PANEL_RAW;
        for (unsigned i = 0; i < 4; ++i) {
            if (!panel_number(t[i + 3], 16, request->op == PANEL_SEG ? 255 : 65535, &value)) return false;
            request->words[i] = request->op == PANEL_SEG ? panel_word(i, (uint8_t)value) : (uint16_t)value;
        }
    } else return false;
    return true;
}

typedef struct {
    uint16_t words[4];
    uint8_t keys;
    uint32_t event_id;
    bool enabled;
} panel_state_t;

static inline void panel_state_init(panel_state_t *state) {
    memset(state, 0, sizeof(*state));
    state->enabled = true;
    for (unsigned i = 0; i < 4; ++i) state->words[i] = panel_word(i, (uint8_t)panel_glyph("1234"[i]));
}

static inline void panel_handle(panel_state_t *state, const panel_request_t *r,
                                char *reply, size_t size) {
    if (r->op == PANEL_PING) {
        snprintf(reply, size, "P1 %u OK PONG\n", r->id);
    } else if (r->op == PANEL_KEYS) {
        snprintf(reply, size, "P1 %u OK KEYS %u %lu\n", r->id, state->keys, (unsigned long)state->event_id);
    } else if (r->op == PANEL_BEEP) {
        snprintf(reply, size, "P1 %u OK BEEP\n", r->id);
    } else {
        const char *name = "ENABLE";
        if (r->op == PANEL_ENABLE) state->enabled = r->enabled != 0;
        else {
            for (unsigned i = 0; i < 4; ++i)
                state->words[i] = r->op == PANEL_BLANK ? panel_word(i, 0) : r->words[i];
            name = r->op == PANEL_SHOW ? "SHOW" : r->op == PANEL_SEG ? "SEG" :
                   r->op == PANEL_RAW ? "RAW" : "BLANK";
        }
        snprintf(reply, size, "P1 %u OK %s\n", r->id, name);
    }
}

typedef struct {
    uint8_t stable, candidate, samples[3];
} panel_debounce_t;

// Called every 5ms. Each key needs four equal samples (20ms) independently.
static inline uint8_t panel_debounce(panel_debounce_t *d, uint8_t raw) {
    uint8_t before = d->stable;
    for (unsigned i = 0; i < PANEL_KEY_COUNT; ++i) {
        uint8_t bit = (uint8_t)(1u << i);
        if ((raw ^ d->candidate) & bit) {
            d->candidate ^= bit;
            d->samples[i] = 1;
        } else if (d->samples[i] < 4) ++d->samples[i];
        if (d->samples[i] >= 4) d->stable = (uint8_t)((d->stable & ~bit) | (raw & bit));
    }
    return before ^ d->stable;
}

enum { PANEL_REPLY_OK, PANEL_REPLY_ERROR, PANEL_REPLY_READY, PANEL_REPLY_KEY };
typedef struct {
    uint16_t id;
    unsigned kind;
    char command[12];
    bool has_keys;
    uint8_t keys;
    uint32_t event_id;
} panel_reply_t;

static inline bool panel_parse_reply(char *line, panel_reply_t *reply) {
    char *t[8];
    uint32_t id, keys, event;
    int n = panel_tokens(line, t, 8);
    memset(reply, 0, sizeof(*reply));
    if (n < 4 || strcmp(t[0], "P1") || !panel_number(t[1], 10, 65535, &id)) return false;
    reply->id = (uint16_t)id;
    if (!id && !strcmp(t[2], "READY") && n == 4 && !strcmp(t[3], "1")) reply->kind = PANEL_REPLY_READY;
    else if (!id && !strcmp(t[2], "KEY") && n == 6 &&
             panel_number(t[3], 10, UINT32_MAX, &event) && panel_number(t[4], 10, 7, &keys)) {
        uint32_t changed;
        if (!panel_number(t[5], 10, 7, &changed)) return false;
        reply->kind = PANEL_REPLY_KEY; reply->has_keys = true;
        reply->keys = (uint8_t)keys; reply->event_id = event;
    } else if (id && !strcmp(t[2], "OK")) {
        reply->kind = PANEL_REPLY_OK;
        if (strlen(t[3]) >= sizeof(reply->command)) return false;
        strcpy(reply->command, t[3]);
        if (!strcmp(t[3], "KEYS")) {
            if (n != 6 || !panel_number(t[4], 10, 7, &keys) ||
                !panel_number(t[5], 10, UINT32_MAX, &event)) return false;
            reply->has_keys = true; reply->keys = (uint8_t)keys; reply->event_id = event;
        } else if (n != 4 || (strcmp(t[3], "PONG") && strcmp(t[3], "SHOW") &&
                   strcmp(t[3], "SEG") && strcmp(t[3], "RAW") &&
                   strcmp(t[3], "BLANK") && strcmp(t[3], "ENABLE") && strcmp(t[3], "BEEP"))) return false;
    } else if (id && !strcmp(t[2], "ERR") && n == 4) reply->kind = PANEL_REPLY_ERROR;
    else return false;
    return true;
}

#endif
