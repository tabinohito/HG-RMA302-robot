#include "cart_direction.hpp"
#include "motor_board_config.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace motor;

static float hz(float rpm) { return board::electrical_hz_from_wheel_rpm(rpm); }

static void recorded_saturation() {
  SpeedLoop speed;
  assert(speed.start(hz(100),2,hz(20),true));
  speed.engage(hz(30.75f),2);
  for(unsigned i=0;i<6000;++i) assert(speed.step(hz(30.75f),true));
  assert(speed.reference>hz(99) && speed.output==2);
  speed.brake();
  assert(speed.output<=0 && speed.reference<=hz(30.751f));
  for(unsigned i=0;i<300;++i) {
    assert(speed.step(hz(30.75f),true));
    assert(speed.output<0 && speed.output>=-.60001f);
  }
  // Reducing a still-nonzero stick target must not spend seconds lowering an
  // unreachable reference either. Acceleration and legacy mode retain slew.
  SpeedLoop reduced;
  assert(reduced.start(hz(100),2,hz(20),true));
  reduced.engage(hz(30.75f),2);
  for(unsigned i=0;i<6000;++i) assert(reduced.step(hz(30.75f),true));
  assert(reduced.set_target(hz(10)));
  assert(reduced.reference<=hz(30.751f));
  assert(reduced.output<=0);
  const float before=reduced.reference;
  assert(reduced.step(hz(30.75f),true));
  assert(std::abs(reduced.reference-(before-.2f))<1e-4f);
  assert(reduced.output<0); // Nonzero reductions discard saturated motoring integral too.
}

// Independent PMSM plant: leave an unreachable 100 RPM request saturated,
// then brake directly instead of first returning to a reachable lower speed.
static void stop_at_voltage_limit(int direction,bool right_motor,bool partial) {
  const Parameters p=right_motor ? Parameters{1.85f,.000930f} : Parameters{1.887f,.000881f};
  Controller c; c.tuning.max_mrpm=100000;
  c.tuning.overspeed_filtered_milli=2000;
  c.tuning.max_slew_mrpm_s=30000;
  c.tuning.accel_ff_ma=300;
  assert(c.run_speed({18,26,4,.2f},p,hz(100),2,hz(30),true,hz(8)));
  constexpr float bus=24.0f;
  float ia=0,ib=0,theta=1.5f,w=0,peak=0,initial=0,slowed=-1,stopped=-1;
  AB physical{},applied{};
  for(unsigned n=0;n<240000;++n) {
    if(n==200000) {
      initial=std::abs(board::wheel_rpm_from_electrical_hz(w/(2*pi)));
      assert(c.state==State::closed && initial>28 && initial<34);
      assert(c.speed_loop.reference>hz(99));
      assert(partial ? c.set_speed_reference(hz(10)) : c.brake_speed());
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
    if(c.fault!=Fault::none) printf("fault=%s reason=%s at %.3f\n",name(c.fault),c.observer_reason,n*dt);
    assert(c.fault==Fault::none);
    assert(c.current_reference.q<=2.0001f && c.current_reference.q>=-.6001f);
    if(n>=200000) {
      const float rpm=std::abs(board::wheel_rpm_from_electrical_hz(w/(2*pi)));
      if(slowed<0 && rpm<.8f*initial) slowed=(n-200000)*dt;
      if(c.state==State::ready) { stopped=(n-200000)*dt; assert(rpm<.3f);break; }
      if(partial && n==239999) {
        assert(std::abs(rpm-10)<.5f);
        stopped=(n-200000)*dt;
      }
    }
  }
  printf("motor=%s direction=%d partial=%d initial=%.2f RPM 20%% reduction=%.3fs end=%.3fs peak=%.3fA\n",
         right_motor ? "right" : "left",direction,partial,initial,slowed,stopped,peak);
  assert(slowed>=0 && slowed<.35f && stopped>0 && peak<4);
  if(partial) assert(c.state==State::closed);
  else assert(stopped<1 && c.state==State::ready && !c.energized());
}

int main() {
  setvbuf(stdout,nullptr,_IONBF,0);
  recorded_saturation();
  for(int direction:{1,-1}) for(bool right:{false,true}) for(bool partial:{false,true})
    stop_at_voltage_limit(direction,right,partial);
  puts("PASS: saturated reference and positive Iq discarded on nonzero reductions and stops, both motors/directions within limits");
}
