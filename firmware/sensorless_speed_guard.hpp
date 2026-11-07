#pragma once
#include <cmath>

namespace motor {
// The PLL's proportional term produces instantaneous speed spikes at low BEMF.
// Speed mode uses its existing 30 ms feedback filter for the 1.5x speed limit.
// A gross 3x raw estimate still trips immediately; legacy modes stay at 1.5x raw.
inline const char* speed_guard(float raw_omega, float feedback_omega,
                               float ceiling, bool speed_mode,
                               float filtered_ratio=1.5f, float raw_ratio=3.f) {
  if (!std::isfinite(raw_omega) || !std::isfinite(ceiling) || ceiling <= 0 ||
      (speed_mode && !std::isfinite(feedback_omega))) return "speed_invalid";
  if (std::abs(raw_omega) > (speed_mode ? raw_ratio : 1.5f)*ceiling) return "speed_gross";
  if (speed_mode && std::abs(feedback_omega) > filtered_ratio*ceiling) return "overspeed";
  return nullptr;
}
}
