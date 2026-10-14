#include "cart_direction.hpp"
#include "motor_board_config.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace motor;

// Independent electrical/mechanical plant: both physical directions, rotor
// initial position and load, not a mock that immediately reports 'closed'.
static float simulate(bool responsive,int direction,float initial_angle,float pairs,bool quantized=false,float acceleration=10,bool loaded=false) {
  const Parameters p{1.887f,.000881f};
  const Limits limits{18,22,4,.2f};
  const float current_limit=responsive ? 2 : 1;
  Controller c;
  const float accel=responsive ? acceleration : 2;
  assert(c.run_speed(limits,p,board::electrical_hz_from_wheel_rpm(8),current_limit,
                    board::electrical_hz_from_wheel_rpm(accel),true,
                    responsive ? board::electrical_hz_from_wheel_rpm(8) : 0));
  float ia=0,ib=0,theta=initial_angle,w=0,closed_at=-1,reached_at=-1,peak_current=0;
  AB physical{},virtual_voltage{};
  unsigned state_ticks=0;
  for(unsigned n=0;n<190000 && c.energized();++n) {
    // Desired speed changes during both align and ramp. Startup must not reset
    // or hold on to an old target. End with 8, then test an 8 -> 15 RPM step.
    if(responsive && (n==2000 || n==10000 || n==16000)) {
      assert(c.set_speed_reference(board::electrical_hz_from_wheel_rpm(n==10000 ? 15 : 8)));
    }
    if(n==100000) assert(c.set_speed_reference(board::electrical_hz_from_wheel_rpm(15)));
    for(unsigned j=0;j<10;++j) {
      constexpr float h=dt/10,flux=.012f,inertia=.00002f;
      const float drag=.000035f+(loaded ? .000245f*std::clamp((n*dt-5)/2,0.f,1.f) : 0);
      const float ea=-flux*w*std::sin(theta),eb=flux*w*std::cos(theta);
      ia+=h*(physical.a-p.resistance*ia-ea)/p.inductance;
      ib+=h*(physical.b-p.resistance*ib-eb)/p.inductance;
      const float iq=-ia*std::sin(theta)+ib*std::cos(theta);
      w+=h*pairs*(1.5f*pairs*flux*iq-drag*w/pairs)/inertia;
      theta=wrap(theta+h*w);
    }
    const float iu=ia,iv=-.5f*ia+.8660254038f*ib,iw=-.5f*ia-.8660254038f*ib;
    peak_current=std::max({peak_current,std::abs(iu),std::abs(iv),std::abs(iw)});
    const auto sensed=[quantized](float value) { return quantized ? std::round(value/.02938f)*.02938f : value; };
    const Duty duty=c.step(cart_current(sensed(iu),sensed(iv),sensed(iw),direction),19.4f,virtual_voltage);
    virtual_voltage=applied_voltage(duty,19.4f);
    physical=applied_voltage(cart_duty(duty,direction),19.4f);
    if(c.state==State::closed && closed_at<0) closed_at=n*dt;
    if(c.state==State::closed) ++state_ticks;
    if(c.state==State::align || c.state==State::ramp)
      assert(std::hypot(c.current_reference.d,c.current_reference.q)<=(responsive ? .60001f : .30001f));
    assert(c.current_reference.q<=current_limit+.00001f);
    if(n>100000 && reached_at<0 && direction*board::wheel_rpm_from_electrical_hz(w/(2*pi))>=14.3f)
      reached_at=n*dt-5;
  }
  const float rpm=board::wheel_rpm_from_electrical_hz(w/(2*pi));
  printf("responsive=%d direction=%d theta0=%.1f pairs=%.0f closed=%.3fs step_to_14.3=%.3fs final=%.3f state=%s fault=%s quantized=%d accel=%.0f peak=%.3fA\n",
         responsive,direction,initial_angle,pairs,closed_at,reached_at,rpm,name(c.state),name(c.fault),quantized,acceleration,peak_current);
  assert(c.state==State::closed && state_ticks>100000 && c.fault==Fault::none);
  assert(std::abs(rpm-direction*15)<.3f);
  assert(peak_current<limits.current);
  if(responsive) { assert(closed_at<2.2f); assert(reached_at>0 && reached_at<1.5f); }
  else assert(reached_at>2.8f);
  if(loaded) assert(peak_current>1.0f);
  return reached_at;
}
int main() {
  setvbuf(stdout,nullptr,_IONBF,0);
  simulate(false,1,0,2);
  for(int direction : {1,-1}) for(float angle : {0.f,1.5f,3.f}) for(float pairs : {2.f,4.f})
    simulate(true,direction,angle,pairs);
  for(int direction : {1,-1}) for(float accel : {10.f,20.f}) simulate(true,direction,1.5f,2,true,accel);
  for(int direction : {1,-1}) simulate(true,direction,1.5f,2,true,10,true);
  SpeedLoop speed;
  assert(speed.start(50,2,50,true)); speed.engage(0,0);
  for(unsigned i=0;i<10000;++i) assert(speed.step(0));
  assert(speed.output==2);
  assert(!speed.start(50,2.01f,50,true));
  Controller c; const Limits l{18,22,2,.2f}; const Parameters p{1.887f,.000881f};
  assert(!c.run_speed(l,p,40,1,30,true,NAN));
  assert(c.run_speed(l,p,40,1,30,true,27));
  assert(c.set_speed_reference(30));
  assert(!c.set_speed_reference(-30));
  c.step({3,0},19,{}); assert(c.fault==Fault::current && !c.set_speed_reference(40));
  c.stop(); assert(!c.set_speed_reference(40));
  puts("PASS: startup/latest target, faster slew under load, mirrored phases, rotor positions, current and fault guards");
}
