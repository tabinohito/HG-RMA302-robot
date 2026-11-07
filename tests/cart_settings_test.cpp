#include "cart_settings.hpp"
#include "current_sum_guard.hpp"
#include "motor_control.hpp"
#include "startup_lock.hpp"
#include "sensorless_speed_guard.hpp"
#include <cassert>
#include <cstdio>

using namespace motor;
static void fill(CartSettingsTransaction& t, const CartSettings& c) {
  for (unsigned i=0;i<CartSettings::field_count;++i) assert(t.put(CartSettings::key(i),c.get(i)));
}
int main() {
  CartSettings c;
  assert(c.valid()); assert(c.electrical_hz(25)>82 && c.electrical_hz(25)<84);
  // Divide by the wire scale: multiplying 26000 by .001f can exceed 26.0f.
  const Limits limits{c.bus_min_mv/1000.f,c.bus_max_mv/1000.f,c.phase_trip_ma/1000.f,.2f};
  assert(limits.valid() && limits.bus_max==26.f);
  CartSettingsTransaction t;
  assert(!t.begin(0)); assert(!t.ready(0));
  assert(t.begin(123)); assert(t.put("ntc_trip_raw",3000));
  assert(!t.ready(123)); // Partial transfer cannot commit.
  assert(!t.put("ntc_trip_raw",3100)); assert(t.failed);
  assert(t.begin(124)); assert(!t.put("unknown",1)); assert(t.failed);
  assert(t.begin(125)); fill(t,c); assert(!t.ready(124)); assert(t.ready(125));
  t.abort(); assert(!t.ready(125));
  c.ntc_trip_raw=3200; c.bus_max_mv=25000; c.max_mrpm=15000;
  assert(t.begin(126)); fill(t,c); assert(t.ready(126));
  for (auto invalid : {0U,999U,3201U,4095U}) {
    auto bad=c; bad.ntc_trip_raw=invalid; assert(!bad.valid());
    assert(t.begin(127)); fill(t,bad); assert(!t.ready(127));
  }
  auto bad=c; bad.bus_max_mv=26001; assert(!bad.valid());
  bad=c; bad.max_iq_ma=2001; assert(!bad.valid());
  bad=c; bad.phase_trip_ma=1000; assert(!bad.valid());
  bad=c; bad.wheel_cycles_milli=1000000; assert(!bad.valid());
  bad=c; bad.sum_hard_ma=bad.sum_soft_ma; assert(!bad.valid());
  bad=c; bad.lock_score=0; assert(!bad.valid());
  bad=c; bad.accel_ff_ma=1001; assert(!bad.valid());
  bad=c; bad.accel_ff_ma=1000; assert(bad.valid());
  bad=c; bad.max_slew_mrpm_s=30000; assert(bad.valid());
  bad=c; bad.max_slew_mrpm_s=30001; assert(!bad.valid());
  bad=c; bad.overspeed_raw_milli=1000; assert(!bad.valid());
  // Settings are actually consumed by the control and protection paths.
  Controller control; control.tuning.startup_ma=400;
  assert(control.run_speed({18,26,4,.2f},{1.887f,.000881f},30,2,60,true,27));
  for (int n=0;n<2100;++n) control.step({},24,{});
  assert(control.state==State::align && control.current_reference.d>.399f && control.current_reference.d<.401f);
  control.stop(); assert(control.tuning.startup_ma==400);
  CurrentSumGuard sum; sum.soft_a=.7f; sum.hard_a=1.8f; sum.sample_limit=2;
  sum.reset(); assert(sum.soft_a==.7f && sum.sample_limit==2);
  assert(sum.step(.8f,0,0)==CurrentSumGuard::Issue::none);
  assert(sum.step(.8f,0,0)==CurrentSumGuard::Issue::sustained);
  sum.reset(); assert(sum.step(1.81f,0,0)==CurrentSumGuard::Issue::excessive);
  assert(startup_lock_score(100,true,true,10,500)==101);
  assert(startup_lock_score(100,false,true,10,500)==90);
  assert(startup_lock_score(500,true,true,10,500)==500);
  assert(speed_guard(15,15,10,true,1.4f,2.f));
  assert(!speed_guard(15,15,10,true,1.6f,2.f));
  puts("PASS: complete atomic settings, bad/incomplete/duplicate values, boundaries, runtime guards and startup current");
}
