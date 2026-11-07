#ifndef POWER_PANEL_LINK_H
#define POWER_PANEL_LINK_H
#include "panel_test.h"
#include "power_config.h"

typedef struct {
    bool online, keys_known, event_known, enabled, enabled_known, acknowledged_enabled;
    bool segments_known, beep_supported;
    uint8_t keys, segments[4], acknowledged_segments[4], sent_segments[4];
    bool sent_enabled;
    uint16_t sequence, pending;
    unsigned pending_op, beep_ms;
    uint32_t sent_at, polled_at, event_id, beep_at;
} power_panel_t;

static inline void power_panel_disconnected(power_panel_t *p, uint32_t now) {
    p->online = p->keys_known = p->event_known = false;
    p->enabled_known = p->segments_known = false;
    p->pending = 0; p->beep_ms = 0; p->sent_at = now - 500;
}

static inline void power_panel_init(power_panel_t *p) {
    memset(p, 0, sizeof(*p));
    p->beep_supported = true;
    power_panel_disconnected(p, 0);
}

static inline void power_panel_beep(power_panel_t *p, unsigned ms, uint32_t now) {
    if (POWER_BUZZER_ENABLE && p->online && p->beep_supported) {
        p->beep_ms = ms; p->beep_at = now;
    }
}

static inline bool power_panel_tick(power_panel_t *p, uint32_t now, char *out, size_t size) {
    if (p->pending) {
        if ((uint32_t)(now - p->sent_at) < 500) return false;
        power_panel_disconnected(p, now);
    }
    if (p->beep_ms && (uint32_t)(now - p->beep_at) >= 1000) p->beep_ms = 0;
    unsigned op;
    if (!p->online) {
        if ((uint32_t)(now - p->sent_at) < 500) return false;
        op = PANEL_PING;
    } else if (!p->enabled && (!p->enabled_known || p->acknowledged_enabled)) op = PANEL_ENABLE;
    else if (!p->segments_known || memcmp(p->segments, p->acknowledged_segments, 4)) op = PANEL_SEG;
    else if (!p->enabled_known || p->enabled != p->acknowledged_enabled) op = PANEL_ENABLE;
    else if (p->beep_ms) op = PANEL_BEEP;
    else if (!p->keys_known || (uint32_t)(now - p->polled_at) >= 100) op = PANEL_KEYS;
    else return false;
    if (++p->sequence == 0) ++p->sequence;
    p->pending = p->sequence; p->pending_op = op; p->sent_at = now;
    if (op == PANEL_SEG) {
        memcpy(p->sent_segments, p->segments, 4);
        snprintf(out, size, "P1 %u SEG %02X %02X %02X %02X\n", p->pending,
                 p->segments[0], p->segments[1], p->segments[2], p->segments[3]);
    } else if (op == PANEL_ENABLE) {
        p->sent_enabled = p->enabled;
        snprintf(out, size, "P1 %u ENABLE %u\n", p->pending, p->sent_enabled ? 1u : 0u);
    } else if (op == PANEL_BEEP) {
        snprintf(out, size, "P1 %u BEEP %u\n", p->pending, p->beep_ms);
        p->beep_ms = 0; // Never replay a pulse after timeout/reconnect.
    } else snprintf(out, size, "P1 %u %s\n", p->pending, op == PANEL_PING ? "PING" : "KEYS");
    return true;
}

static inline uint8_t power_panel_reply(power_panel_t *p, const panel_reply_t *r, uint32_t now) {
    if (r->kind == PANEL_REPLY_READY) {
        power_panel_disconnected(p, now); p->beep_supported = true; return 0;
    }
    if (r->kind != PANEL_REPLY_KEY) {
        if (!p->pending || r->id != p->pending) return 0;
        if (r->kind == PANEL_REPLY_ERROR) {
            if (p->pending_op == PANEL_BEEP) { p->pending = 0; p->beep_supported = false; }
            else power_panel_disconnected(p, now);
            return 0;
        }
        const char *expected = p->pending_op == PANEL_PING ? "PONG" :
            p->pending_op == PANEL_SEG ? "SEG" : p->pending_op == PANEL_ENABLE ? "ENABLE" :
            p->pending_op == PANEL_BEEP ? "BEEP" : "KEYS";
        if (r->kind != PANEL_REPLY_OK || strcmp(r->command, expected)) return 0;
        if (p->pending_op == PANEL_PING) p->online = true;
        if (p->pending_op == PANEL_SEG) {
            memcpy(p->acknowledged_segments, p->sent_segments, 4); p->segments_known = true;
        }
        if (p->pending_op == PANEL_ENABLE) {
            p->acknowledged_enabled = p->sent_enabled; p->enabled_known = true;
        }
        if (p->pending_op == PANEL_KEYS) p->polled_at = now;
        p->pending = 0;
    }
    if (!p->online || !r->has_keys) return 0;
    if (p->event_known && (int32_t)(r->event_id - p->event_id) < 0) return 0;
    uint8_t pressed = p->keys_known ? (uint8_t)(r->keys & ~p->keys) : 0;
    p->keys = r->keys; p->keys_known = p->event_known = true; p->event_id = r->event_id;
    return pressed;
}
#endif
