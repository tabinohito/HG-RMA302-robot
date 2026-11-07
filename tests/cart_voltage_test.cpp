#include "motor_control.hpp"
#include "motor_board_config.hpp"
#include <cassert>
#include <cstdio>
#include <limits>

using namespace motor;

int main() {
  const Limits limits{board::bench_bus_min, board::bench_bus_max,
                      board::bench_phase_trip, board::bench_identify_voltage};
  const Parameters p{1.887f, .000881f};
  assert(limits.valid());
  assert(limits.bus_min == 18 && limits.bus_max == 26 && limits.current == 4);
  auto start = [&](Controller& c) {
    assert(c.run_speed(limits, p, board::electrical_hz_from_wheel_rpm(15), 2,
                       board::electrical_hz_from_wheel_rpm(20), true,
                       board::electrical_hz_from_wheel_rpm(8)));
  };
  for (float bus : {18.f, 19.4f, 25.f, 25.2f, 26.f}) {
    Controller c; start(c);
    c.step({}, bus, {});
    assert(c.fault == Fault::none && c.energized());
    assert(std::abs(c.current_reference.q) <= 2);
    c.stop(); assert(!c.energized());
  }
  for (float bus : {17.99f, 26.01f, 30.f}) {
    Controller c; start(c);
    c.step({}, bus, {});
    assert(c.fault == Fault::bus && !c.energized());
  }
  Controller c; start(c);
  c.step({4.1f, 0}, 25.2f, {});
  assert(c.fault == Fault::current && !c.energized());
  for (float maximum : {26.01f, 30.f, std::numeric_limits<float>::infinity()}) {
    Limits invalid = limits; invalid.bus_max = maximum;
    assert(!invalid.valid());
  }
  puts("PASS: 6S charged voltage accepted, 18/26 V boundaries, voltage/current trips preserved");
}
