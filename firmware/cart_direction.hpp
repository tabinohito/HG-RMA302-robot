#pragma once

#include <cmath>
#include <cstdint>
#include "motor_control.hpp"

namespace motor {
// Keep the proven positive-speed observer in a virtual U/V/W frame. Reflect
// BOTH sensing and actuation for reverse: U/V/W physical -> U/W/V virtual.
// The previous/queued duty used by the observer stays in the virtual frame.
inline AB cart_current(float u, float v, float w, int direction) {
  return direction < 0 ? clarke(u, w, v) : clarke(u, v, w);
}
inline Duty cart_duty(Duty virtual_duty, int direction) {
  if (direction < 0) return {virtual_duty.u, virtual_duty.w, virtual_duty.v};
  return virtual_duty;
}

class CartDirection {
 public:
  static constexpr uint32_t reverse_dwell_ms = 2000;
  bool begin(int requested, uint32_t now) {
    if ((requested != 1 && requested != -1) || active_) return false;
    if (last_ && requested != last_ && now - stopped_at_ < reverse_dwell_ms) return false;
    direction = last_ = requested;
    active_ = true;
    return true;
  }
  void stop(uint32_t now) {
    // Repeated stop commands must not indefinitely extend the dwell.
    if (active_) { stopped_at_ = now; active_ = false; }
  }
  // Caller must have completed controlled deceleration and low-speed settling.
  void confirm_braked() { if (!active_) last_=0; }
  bool accepts(float iq) const {
    return active_ && std::isfinite(iq) &&
        (iq == 0 || (iq > 0 ? 1 : -1) == direction);
  }
  static int start_direction(float hz, float iq) {
    if (!std::isfinite(hz) || !std::isfinite(iq) || hz == 0 || iq == 0 ||
        (hz > 0) != (iq > 0)) return 0;
    return hz > 0 ? 1 : -1;
  }
  int direction = 1;
 private:
  int last_ = 0;
  bool active_ = false;
  uint32_t stopped_at_ = 0;
};
}
