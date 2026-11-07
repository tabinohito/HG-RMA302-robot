#ifndef CART_HOST_CONTROL_H
#define CART_HOST_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#define CART_HOST_WATCHDOG_MS 750u

typedef enum { CART_BOOT, CART_USB_ONLY, CART_WAIT_POWER, CART_READY,
    CART_DRIVING, CART_RECOVERING, CART_STOPPED, CART_FAULT } cart_state_t;
static const char *const cart_state_names[] = {
    "BOOT", "USB_ONLY", "WAIT_POWER", "READY", "DRIVING", "RECOVERING", "STOPPED", "FAULT"};
typedef struct {
    uint32_t session, sequence, seen_at;
    bool managed, estop, joystick;
    cart_state_t state;
    const char *stop_reason;
} cart_host_t;

static inline bool cart_u32(const char *s, uint32_t *out) {
    if (!s || *s < '0' || *s > '9') return false;
    char *end; errno=0; unsigned long n=strtoul(s,&end,10);
    if (errno || *end || n>UINT32_MAX) return false;
    *out=(uint32_t)n; return true;
}
static inline bool cart_lease_alive(const cart_host_t *h, uint32_t now) {
    return h->managed && (uint32_t)(now-h->seen_at)<CART_HOST_WATCHDOG_MS;
}
static inline void cart_stop(cart_host_t *h, const char *reason) {
    h->estop=true; h->state=CART_STOPPED; h->stop_reason=reason;
}
static inline bool cart_claim(cart_host_t *h, uint32_t session, uint32_t now) {
    if (!session || (h->session!=session && cart_lease_alive(h,now))) return false;
    if(h->session!=session) h->sequence=0;
    h->session=session; h->managed=true; h->seen_at=now;
    h->state=h->estop ? CART_STOPPED : CART_BOOT;
    if(!h->stop_reason) h->stop_reason="none";
    return true;
}
static inline bool cart_heartbeat(cart_host_t *h, uint32_t session, uint32_t sequence,
                                  const char *state, uint32_t now) {
    if(!h->managed || session!=h->session || sequence<=h->sequence) return false;
    unsigned index=0;
    while(index<sizeof(cart_state_names)/sizeof(*cart_state_names) && strcmp(state,cart_state_names[index])) ++index;
    if(index==sizeof(cart_state_names)/sizeof(*cart_state_names)) return false;
    h->seen_at=now; h->sequence=sequence;
    h->state=h->estop ? CART_STOPPED : (cart_state_t)index;
    return true;
}
static inline bool cart_clear(cart_host_t *h,uint32_t session,uint32_t now,bool button_down,bool relay) {
    if(session!=h->session || !cart_lease_alive(h,now) || button_down || relay) return false;
    h->estop=false; h->state=CART_WAIT_POWER; h->stop_reason="none";
    return true; // Clearing never switches the SSR on.
}
static inline bool cart_can_power_on(const cart_host_t *h,uint32_t now) {
    return !h->estop && (!h->managed || cart_lease_alive(h,now));
}
static inline bool cart_watchdog(cart_host_t *h,uint32_t now,bool connected) {
    if(h->managed && (!connected || !cart_lease_alive(h,now))) {
        bool changed=!h->estop;
        cart_stop(h,connected ? "host_timeout" : "usb_disconnect");
        return changed;
    }
    return false;
}
#endif
