#pragma once

#include <algorithm>
#include <cmath>

namespace motor {

// Monitor three-shunt consistency separately from the instantaneous per-phase
// overcurrent and ADC-validity checks. At 20 kHz, ten samples span 0.5 ms.
struct CurrentSumGuard {
  static constexpr float soft_limit_a = 1.2f;
  static constexpr float hard_limit_a = 2.0f;
  static constexpr unsigned trip_samples = 10;
  enum class Issue { none, sustained, excessive, nonfinite };
  float sum_a = 0, peak_a = 0;
  unsigned consecutive = 0, peak_consecutive = 0;
  float soft_a = soft_limit_a, hard_a = hard_limit_a;
  unsigned sample_limit = trip_samples;

  void reset() { sum_a=peak_a=0; consecutive=peak_consecutive=0; }

  Issue step(float u, float v, float w) {
    sum_a = u + v + w;
    if (!std::isfinite(sum_a)) return Issue::nonfinite;
    const float magnitude = std::abs(sum_a);
    peak_a = std::max(peak_a, magnitude);
    if (magnitude > hard_a) return Issue::excessive;
    if (magnitude > soft_a) {
      if (consecutive < sample_limit) ++consecutive;
    } else {
      consecutive = 0;
    }
    peak_consecutive = std::max(peak_consecutive, consecutive);
    return consecutive >= sample_limit ? Issue::sustained : Issue::none;
  }

  static const char* name(Issue issue) {
    switch (issue) {
      case Issue::sustained: return "current_sum_sustained";
      case Issue::excessive: return "current_sum_hard";
      case Issue::nonfinite: return "current_sum_nonfinite";
      default: return "none";
    }
  }
};

} // namespace motor
