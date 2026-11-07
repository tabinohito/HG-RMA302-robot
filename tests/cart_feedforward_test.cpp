#include "cart_direction.hpp"
#include "motor_board_config.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace motor;
static float hz(float rpm) { return board::electrical_hz_from_wheel_rpm(rpm); }

static void limits_and_retargeting() {
  SpeedLoop s;
  assert(!s.start(hz(8),2,hz(20),true,NAN));
  assert(!s.start(hz(8),2,hz(20),true,1.001f));
  assert(s.start(hz(8),2,hz(20),true,.3f)); s.engage(hz(8),.1f);
  float previous=0,mean=0;
  // A slowly moving stick arrives as 50 Hz stair steps. FF must not become
  // large 50 Hz torque pulses even though the internal slew is faster.
  for (int n=0;n<1000;++n) {
    if(n%20==0) assert(s.set_target(hz(8+(n/20+1)*.1f)));
    assert(s.step(s.reference,true));
    assert(std::abs(s.feedforward-previous)<.011f);
    assert(s.feedforward>=0 && s.feedforward<=.30001f);
    previous=s.feedforward;
    if(n>=500) mean+=s.feedforward/500;
  }
  assert(mean>.07f && mean<.08f); // 5 RPM/s is 1/4 of the configured 20.
  assert(s.set_target(hz(8))); assert(s.feedforward==0);
  assert(s.step(hz(13),true)); assert(s.feedforward==0);
  assert(s.set_target(hz(30)));
  for(int n=0;n<300;++n) assert(s.step(hz(8),true));
  assert(s.feedforward>.29f && s.output<=2);
  s.brake(); assert(s.feedforward==0 && s.output<=0);
  for(int n=0;n<500;++n) {assert(s.step(hz(8),true));assert(s.feedforward==0 && s.output>=-.60001f && s.output<=0);}
  assert(s.start(hz(25),.1f,hz(20),true,1)); s.engage(hz(8),0);
  for(int n=0;n<500;++n) {assert(s.step(hz(8),true));assert(s.output<=.10001f && s.feedforward<=.10001f);}
  // Legacy and startup/blend paths do not gain acceleration FF.
  assert(s.start(hz(25),2,hz(20),false,1)); s.engage(hz(8),0);
  for(int n=0;n<200;++n) {assert(s.step(hz(8),true));assert(s.feedforward==0);}
  assert(s.start(hz(25),2,hz(20),true,1)); s.engage(hz(8),0);
  for(int n=0;n<200;++n) {assert(s.step(hz(8),false));assert(s.feedforward==0);}
}

// Independent PMSM plant, both motor constants/directions, two inertias and a
// changing drag load. These are model checks, not a ground traction calibration.
static float plant(int direction,bool right,bool heavy,unsigned ff_ma) {
  const Parameters p=right ? Parameters{1.85f,.000930f} : Parameters{1.887f,.000881f};
  Controller c; c.tuning.max_mrpm=100000; c.tuning.startup_ma=1000; c.tuning.accel_ff_ma=ff_ma;
  c.tuning.max_slew_mrpm_s=30000;
  c.tuning.overspeed_filtered_milli=2000;
  assert(c.run_speed({18,26,4,.2f},p,hz(8),2,hz(30),true,hz(8)));
  float ia=0,ib=0,theta=1.5f,w=0,peak=0,closed=-1,ready=-1,error=0,ff_peak=0,max_rpm=0;
  AB physical{},applied{};
  for(unsigned n=0;n<180000;++n) {
    if(n==60000) assert(c.set_speed_reference(hz(25)));
    if(n==110000) { assert(c.set_speed_reference(hz(10))); assert(c.speed_loop.feedforward==0); }
    if(n==140000) { assert(c.brake_speed()); assert(c.speed_loop.feedforward==0); }
    for(unsigned j=0;j<10;++j) {
      constexpr float h=dt/10,flux=.012f,pairs=2;
      const float inertia=heavy ? .00004f : .00002f;
      const float drag=.000035f+(heavy ? .00010f*std::clamp((n*dt-3)/.5f,0.f,1.f) : 0);
      ia+=h*(physical.a-p.resistance*ia+flux*w*std::sin(theta))/p.inductance;
      ib+=h*(physical.b-p.resistance*ib-flux*w*std::cos(theta))/p.inductance;
      const float iq=-ia*std::sin(theta)+ib*std::cos(theta);
      w+=h*pairs*(1.5f*pairs*flux*iq-drag*w/pairs)/inertia;
      theta=wrap(theta+h*w);
    }
    const float u=ia,v=-.5f*ia+.8660254038f*ib,z=-.5f*ia-.8660254038f*ib;
    peak=std::max({peak,std::abs(u),std::abs(v),std::abs(z)});
    const auto sense=[](float i) { return std::round(i/.02938f)*.02938f; };
    const auto duty=c.step(cart_current(sense(u),sense(v),sense(z),direction),24,applied);
    applied=applied_voltage(duty,24);physical=applied_voltage(cart_duty(duty,direction),24);
    const float rpm=std::abs(board::wheel_rpm_from_electrical_hz(w/(2*pi)));
    max_rpm=std::max(max_rpm,rpm);ff_peak=std::max(ff_peak,c.speed_loop.feedforward);
    if(c.state==State::closed && closed<0) closed=n*dt;
    if(c.fault!=Fault::none) printf("fault=%s reason=%s at %.3fs\n",name(c.fault),c.observer_reason,n*dt);
    assert(c.fault==Fault::none);
    assert(c.current_reference.q<=2.00001f && c.current_reference.q>=-.60001f);
    if(n>=60000 && n<77000) error+=std::abs(board::wheel_rpm_from_electrical_hz(c.speed_loop.reference)-rpm)*dt;
    if(n==109999) assert(std::abs(rpm-25)<.5f);
    if(n==139999) assert(std::abs(rpm-10)<.5f);
    if(c.state==State::ready) {ready=n*dt;assert(rpm<.3f);break;}
  }
  printf("direction=%d motor=%s heavy=%d ff=%umA lock=%.3fs accel_error=%.3f peak=%.3fA max=%.2fRPM stop=%.3fs\n",
         direction,right?"right":"left",heavy,ff_ma,closed,error,peak,max_rpm,ready-7);
  assert(closed>0 && closed<2.5f && ready>=7 && ready<8 && peak<4 && max_rpm<28);
  assert(ff_ma ? ff_peak>.25f : ff_peak==0);
  return error;
}
int main() {
  setvbuf(stdout,nullptr,_IONBF,0);limits_and_retargeting();
  for(int direction:{1,-1}) for(bool right:{false,true}) for(bool heavy:{false,true}) {
    const float baseline=plant(direction,right,heavy,0);
    const float with_ff=plant(direction,right,heavy,300);
    // Added torque should actually improve tracking under the heavier plant.
    if(heavy) assert(with_ff<baseline);
  }
  puts("PASS: bounded FF, UART smoothing, immediate reduction/brake cancellation, load/direction models");
}
