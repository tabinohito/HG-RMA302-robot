#include "sensorless_speed_guard.hpp"
#include "motor_speed.hpp"
#include <cassert>
#include <cstdio>
#include <limits>

int main() {
  using namespace motor;
  constexpr float hz=8.f*199/60;
  SpeedLoop speed;
  assert(speed.start(hz,2,60,true)); speed.engage(hz,0.1f);
  // Recorded right-wheel spike: raw ~42 Hz while averaged speed stays ~26.5 Hz.
  assert(speed.observe(42.1f));
  assert(!speed_guard(42.1f,speed.feedback,hz,true));
  assert(speed_guard(42.1f,speed.feedback,hz,false)); // legacy protection unchanged
  for (int n=0;n<1000;++n) {
    const float raw=hz*(n%2 ? .4f : 1.6f);
    assert(speed.observe(raw));
    assert(!speed_guard(raw,speed.feedback,hz,true));
  }
  // A real 2x overspeed is detected within 25 ms, despite filtering.
  unsigned elapsed=0;
  do {
    assert(speed.observe(2*hz)); ++elapsed;
  } while (!speed_guard(2*hz,speed.feedback,hz,true) && elapsed<100);
  assert(elapsed>0 && elapsed<=25);
  assert(speed_guard(3.01f*hz,hz,hz,true));
  assert(speed_guard(-3.01f*hz,hz,hz,true));
  assert(speed_guard(hz,1.51f*hz,hz,true));
  assert(speed_guard(hz,hz,0,true));
  assert(speed_guard(std::numeric_limits<float>::quiet_NaN(),hz,hz,true));
  assert(speed_guard(hz,std::numeric_limits<float>::infinity(),hz,true));
  // Recorded nonzero deceleration trips at 1.5x. Configured 2x margin admits
  // this transient while still rejecting sustained 2.01x and raw 3.01x speed.
  assert(speed_guard(23.081f,23.081f,15.355f,true,1.5f,3));
  assert(!speed_guard(23.081f,23.081f,15.355f,true,2,3));
  assert(speed_guard(hz,2.01f*hz,hz,true,2,3));
  assert(speed_guard(3.01f*hz,hz,hz,true,2,3));
  puts("PASS: recorded PLL spike, noisy steady speed, sustained/gross overspeed, invalid input, legacy limit");
}
