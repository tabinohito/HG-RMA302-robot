#include "cart_direction.hpp"
#include "motor_board_config.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace motor;

// An independent physical PMSM model; reverse exchanges real V/W phases on
// both sensing and PWM. Run long enough to distinguish UART continuous mode
// from the retained 10-second bench timeout.
static void simulate(int direction, bool continuous, float current_limit) {
  const Limits limits{18,22,2,.2f};
  const Parameters p{1.911f,.000949f};
  Controller c;
  assert(c.run_speed(limits,p,board::electrical_hz_from_wheel_rpm(10),current_limit,
                     board::electrical_hz_from_wheel_rpm(2),continuous));
  assert(!c.set_speed_reference(40)); // no change during blind startup
  float ia=0,ib=0,theta=0,w=0,peak=0;
  AB physical{},virt{};
  unsigned closed_count=0;
  for(unsigned n=0;n<450000 && c.energized();++n) {
    if(n==120000) assert(c.set_speed_reference(board::electrical_hz_from_wheel_rpm(8)));
    if(n==210000) assert(c.set_speed_reference(board::electrical_hz_from_wheel_rpm(10)));
    const float drag=n<170000 ? .000015f : .000035f;
    for(unsigned j=0;j<10;++j) {
      constexpr float h=dt/10,flux=.012f,inertia=.00002f,pairs=2;
      const float ea=-flux*w*std::sin(theta),eb=flux*w*std::cos(theta);
      ia+=h*(physical.a-p.resistance*ia-ea)/p.inductance;
      ib+=h*(physical.b-p.resistance*ib-eb)/p.inductance;
      const float iq=-ia*std::sin(theta)+ib*std::cos(theta);
      w+=h*pairs*(1.5f*pairs*flux*iq-drag*w/pairs)/inertia;
      theta=wrap(theta+h*w);
    }
    const float iu=ia,iv=-.5f*ia+.8660254038f*ib,iw=-.5f*ia-.8660254038f*ib;
    const Duty duty=c.step(cart_current(iu,iv,iw,direction),19.4f,virt);
    virt=applied_voltage(duty,19.4f);
    physical=applied_voltage(cart_duty(duty,direction),19.4f);
    if(c.state==State::align || c.state==State::ramp)
      assert(std::hypot(c.current_reference.d,c.current_reference.q)<=.30001f);
    if(c.state==State::closed) {
      ++closed_count;
      assert(c.current_reference.q>=0 && c.current_reference.q<=current_limit);
      peak=std::max(peak,c.current_reference.q);
    }
    if(n==209999) assert(std::abs(board::wheel_rpm_from_electrical_hz(w/(2*pi))-direction*8)<.3f);
  }
  const float rpm=board::wheel_rpm_from_electrical_hz(w/(2*pi));
  printf("direction=%d continuous=%d current_limit=%g state=%s fault=%s closed_s=%.2f final_RPM=%.3f peak_Iq=%.3f\n",
         direction,continuous,current_limit,name(c.state),name(c.fault),closed_count*dt,rpm,peak);
  if(continuous) {
    assert(c.state==State::closed && c.fault==Fault::none && closed_count*dt>15);
    assert(std::abs(rpm-direction*10)<.3f);
  } else assert(c.fault==Fault::timeout && !c.energized());
  c.stop(); assert(!c.speed_mode() && !c.energized() && c.speed_loop.output==0);
}

int main() {
  setvbuf(stdout,nullptr,_IONBF,0);
  for(int direction : {1,-1}) for(float current : {.3f,1.f}) simulate(direction,true,current);
  simulate(1,false,1);
  Controller c; const Parameters p{1.9f,.0009f}; const Limits l{18,22,2,.2f};
  assert(!c.run_speed(l,p,40,1.01f,6,true));
  assert(c.run_speed(l,p,40,1,6,true));
  c.step({3,0},19,{}); assert(c.fault==Fault::current);
  c.stop(); assert(c.run_speed(l,p,40,1,6,true));
  c.step({},23,{}); assert(c.fault==Fault::bus);
  puts("PASS: signed wheel speed, 10->8->10 RPM, load step, >15s closed loop, 1A limit/0.3A startup, bench timeout, protection");
}
