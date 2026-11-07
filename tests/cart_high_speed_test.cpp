#include "cart_direction.hpp"
#include "motor_board_config.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace motor;

// Exercise an unreachable 100 RPM request with the actual 6S voltage/duty
// limits, then return to 25 RPM and brake. Do not mistake acceptance for speed.
static void saturated_request(int direction) {
  const Parameters p{1.887f,.000881f};
  Controller c; c.tuning.max_mrpm=100000;
  const auto hz=[](float rpm) { return board::electrical_hz_from_wheel_rpm(rpm); };
  assert(c.run_speed({18,26,4,.2f},p,hz(100),2,hz(20),true,hz(8)));
  float ia=0,ib=0,theta=1.5f,w=0,peak=0,high_rpm=0,low_rpm=0;
  AB physical{},applied{};
  constexpr float bus=25.2f;
  bool closed=false,ready=false;
  for (unsigned n=0;n<360000;++n) {
    if(n==200000) {
      high_rpm=board::wheel_rpm_from_electrical_hz(w/(2*pi));
      assert(c.state==State::closed && c.set_speed_reference(hz(25)));
    }
    if(n==320000) {
      low_rpm=board::wheel_rpm_from_electrical_hz(w/(2*pi));
      assert(c.brake_speed());
    }
    for(unsigned j=0;j<10;++j) {
      constexpr float h=dt/10,flux=.012f,inertia=.00002f,pairs=2,drag=.000035f;
      ia+=h*(physical.a-p.resistance*ia+flux*w*std::sin(theta))/p.inductance;
      ib+=h*(physical.b-p.resistance*ib-flux*w*std::cos(theta))/p.inductance;
      const float iq=-ia*std::sin(theta)+ib*std::cos(theta);
      w+=h*pairs*(1.5f*pairs*flux*iq-drag*w/pairs)/inertia;
      theta=wrap(theta+h*w);
    }
    const float u=ia,v=-.5f*ia+.8660254038f*ib,z=-.5f*ia-.8660254038f*ib;
    peak=std::max({peak,std::abs(u),std::abs(v),std::abs(z)});
    const auto sense=[](float i) { return std::round(i/.02938f)*.02938f; };
    const auto duty=c.step(cart_current(sense(u),sense(v),sense(z),direction),bus,applied);
    applied=applied_voltage(duty,bus);
    physical=applied_voltage(cart_duty(duty,direction),bus);
    if(c.fault!=Fault::none) {
      printf("FAIL at %.3fs %s/%s rpm=%.2f\n",n*dt,name(c.fault),c.observer_reason,
             board::wheel_rpm_from_electrical_hz(w/(2*pi)));
      assert(false);
    }
    assert(std::abs(c.current_reference.q)<=2.0001f);
    if(c.state==State::align || c.state==State::ramp) assert(std::hypot(c.current_reference.d,c.current_reference.q)<=.6001f);
    closed|=c.state==State::closed;
    if(c.state==State::ready) {ready=true;break;}
  }
  printf("100 RPM request dir=%d voltage-limited speed=%.2f return25=%.2f peak=%.3fA ready=%d\n",direction,high_rpm,low_rpm,peak,ready);
  assert(closed && ready && !c.energized() && peak<4);
  assert(direction*high_rpm>25 && direction*high_rpm<100);
  assert(std::abs(low_rpm-direction*25)<.5f);
  assert(std::abs(board::wheel_rpm_from_electrical_hz(w/(2*pi)))<.3f);
}
int main() {
  setvbuf(stdout,nullptr,_IONBF,0);
  CartSettings settings;settings.max_mrpm=100000;assert(settings.valid());
  settings.max_mrpm=100001;assert(!settings.valid());
  settings.max_mrpm=100000;settings.wheel_cycles_milli=217000;assert(!settings.valid());
  SpeedLoop legacy;assert(!legacy.start(331.6667f,2,66));assert(legacy.start(90,2,66));
  assert(!legacy.set_target(101));
  SpeedLoop cart;assert(cart.start(331.6667f,2,66,true));assert(cart.set_target(360));
  assert(!cart.set_target(360.001f));assert(!cart.start(361,2,66,true));
  assert(!cart.start(331.6667f,2.01f,66,true));
  for(int direction:{1,-1}) saturated_request(direction);
  puts("PASS: 100 RPM command bounds, legacy bounds, 6S voltage saturation, 2A cap, startup, target reduction and stop");
}
