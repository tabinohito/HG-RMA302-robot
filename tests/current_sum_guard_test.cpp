#include "current_sum_guard.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

using Guard = motor::CurrentSumGuard;
using Issue = Guard::Issue;

int main() {
  Guard guard;
  // Captured startup frames previously tripped the 0.6 A single-sample rule.
  for (int n = 0; n < 2000; ++n) {
    assert(guard.step(.402f, .232f, -.021f) == Issue::none);
    assert(guard.step(.545f, .225f, -.120f) == Issue::none);
  }
  assert(guard.consecutive == 0 && std::abs(guard.peak_a-.650f) < .001f);
  for (float sign : {-1.f, 1.f}) {
    guard.reset();
    // Repeated sub-0.5 ms bursts separated by a valid sample do not accumulate.
    for (int burst = 0; burst < 100; ++burst) {
      for (int n = 0; n < 9; ++n)
        assert(guard.step(sign*.7f, sign*.4f, sign*.2f) == Issue::none);
      assert(guard.step(.8f, -.3f, -.5f) == Issue::none);
    }
    assert(guard.peak_consecutive == 9);
    for (int n = 0; n < 9; ++n)
      assert(guard.step(sign*.7f, sign*.4f, sign*.2f) == Issue::none);
    assert(guard.step(sign*.7f, sign*.4f, sign*.2f) == Issue::sustained);
    guard.reset();
    assert(guard.step(sign*.8f, sign*.8f, sign*.5f) == Issue::excessive);
  }
  guard.reset();
  assert(guard.step(1.2f, 0, 0) == Issue::none && guard.consecutive == 0);
  assert(guard.step(2.f, 0, 0) == Issue::none && guard.consecutive == 1);
  guard.reset();
  for (int n = 0; n < 4000; ++n) {
    const float angle = n*.031f;
    assert(guard.step(2*std::sin(angle), 2*std::sin(angle-2.0943951f),
                      2*std::sin(angle+2.0943951f)) == Issue::none);
  }
  for (float bad : {std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::infinity()}) {
    guard.reset(); assert(guard.step(bad, 0, 0) == Issue::nonfinite);
    guard.reset(); assert(guard.step(0, bad, 0) == Issue::nonfinite);
    guard.reset(); assert(guard.step(0, 0, bad) == Issue::nonfinite);
  }
  guard.reset();
  assert(guard.consecutive == 0 && guard.peak_consecutive == 0 && guard.peak_a == 0);
  puts("PASS: recorded startup frames, balanced currents, short bursts, sustained and hard faults, nonfinite, reset");
}
