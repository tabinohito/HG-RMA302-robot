#pragma once
#include <cstdint>

namespace motor {
// Keep 100 ms of net positive evidence at 20 kHz. One invalid observer sample
// costs 5 valid samples, requiring >83.3% valid observations to make progress.
// Speed/angle mismatch always resets all evidence. Runtime loss-of-lock limits
// remain in Controller; this only qualifies the I/f -> FOC handover.
inline constexpr uint32_t startup_lock_bad_cost = 5;
inline constexpr uint32_t startup_lock_required_score = 2000;
inline uint32_t startup_lock_score(uint32_t previous, bool observed, bool geometry_ok,
                                  uint32_t bad_cost=startup_lock_bad_cost,
                                  uint32_t required=startup_lock_required_score) {
  if (!geometry_ok) return 0;
  if (observed) return previous < required ? previous + 1 : required;
  return previous > bad_cost ? previous - bad_cost : 0;
}
}
