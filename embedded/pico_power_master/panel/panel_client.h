#ifndef PANEL_CLIENT_H
#define PANEL_CLIENT_H
#include "panel_test.h"

typedef struct {
    bool online, dirty, keys_known;
    uint8_t keys;
    unsigned value, sent_value;
    uint16_t sequence, pending;
    unsigned pending_op;
    uint32_t sent_at, polled_at;
} panel_client_t;

static inline void panel_client_init(panel_client_t *c) {
    memset(c, 0, sizeof(*c));
    c->dirty = true;
    c->sent_at = UINT32_MAX - 500;
}

static inline void panel_client_show(panel_client_t *c, unsigned value) {
    c->value = value % 10000;
    c->dirty = true;
}

// At most one request in flight. Timeout/reboot returns to PING before commands.
static inline bool panel_client_tick(panel_client_t *c, uint32_t now, char *out, size_t size) {
    if (c->pending) {
        if ((uint32_t)(now - c->sent_at) < 500) return false;
        c->pending = 0; c->online = false; c->keys_known = false; c->dirty = true;
    }
    unsigned op;
    if (!c->online) {
        if ((uint32_t)(now - c->sent_at) < 500) return false;
        op = PANEL_PING;
    } else if (c->dirty) op = PANEL_SHOW;
    else if ((uint32_t)(now - c->polled_at) >= 100) op = PANEL_KEYS;
    else return false;
    if (++c->sequence == 0) ++c->sequence;
    c->pending = c->sequence; c->pending_op = op; c->sent_at = now;
    if (op == PANEL_SHOW) {
        c->sent_value = c->value;
        snprintf(out, size, "P1 %u SHOW %04u\n", c->pending, c->sent_value);
    } else snprintf(out, size, "P1 %u %s\n", c->pending, op == PANEL_PING ? "PING" : "KEYS");
    return true;
}

// Return new presses. A first snapshot (including a held key) establishes state.
static inline uint8_t panel_client_reply(panel_client_t *c, const panel_reply_t *r, uint32_t now) {
    if (r->kind == PANEL_REPLY_READY) {
        c->online = false; c->pending = 0; c->dirty = true; c->keys_known = false;
        c->sent_at = now - 500;
        return 0;
    }
    if (r->kind != PANEL_REPLY_KEY) {
        if (!c->pending || r->id != c->pending) return 0;
        if (r->kind == PANEL_REPLY_ERROR) {
            c->pending = 0; c->online = false; c->keys_known = false; c->dirty = true;
            return 0;
        }
        const char *expected = c->pending_op == PANEL_PING ? "PONG" :
                               c->pending_op == PANEL_SHOW ? "SHOW" : "KEYS";
        if (r->kind != PANEL_REPLY_OK || strcmp(r->command, expected)) return 0;
        if (c->pending_op == PANEL_PING) c->online = true;
        if (c->pending_op == PANEL_SHOW && c->value == c->sent_value) c->dirty = false;
        if (c->pending_op == PANEL_KEYS) c->polled_at = now;
        c->pending = 0;
    }
    if (!r->has_keys || !c->online) return 0;
    uint8_t pressed = c->keys_known ? (uint8_t)(r->keys & ~c->keys) : 0;
    c->keys = r->keys; c->keys_known = true;
    if (pressed & 4) panel_client_show(c, 0); // GPIO19: clear
    else if ((pressed & 3) == 1) panel_client_show(c, c->value + 1); // GPIO22: +1
    else if ((pressed & 3) == 2) panel_client_show(c, c->value + 9999); // GPIO21: -1
    return pressed;
}
#endif
