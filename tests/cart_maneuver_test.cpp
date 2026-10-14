#include "cart_direction.hpp"
#include "motor_board_config.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace motor;

static void simulate(int direction, float initial_angle, float brake_at=4.f) {
  const Parameters p{1.887f,.000881f}; const Limits limits{18,22,4,.2f};
  Controller c; int physical_direction=direction;
  auto start=[&](float rpm) {
    assert(c.run_speed(limits,p,board::electrical_hz_from_wheel_rpm(rpm),2,
                      board::electrical_hz_from_wheel_rpm(20),true,
                      board::electrical_hz_from_wheel_rpm(8)));
  };
  start(20);
  float ia=0,ib=0,theta=initial_angle,w=0,peak=0,ready_at=-1,closed_reverse=-1,first_closed=-1;
  AB physical{},virtual_voltage{};
  bool braked=false,reversed=false;
  for(unsigned n=0;n<240000;++n) {
    if (!braked && n*dt>=brake_at) { assert(c.brake_speed()); braked=true; }
    for(unsigned j=0;j<10;++j) {
      constexpr float h=dt/10,flux=.012f,inertia=.00002f,pairs=2,drag=.000035f;
      const float ea=-flux*w*std::sin(theta),eb=flux*w*std::cos(theta);
      ia+=h*(physical.a-p.resistance*ia-ea)/p.inductance;
      ib+=h*(physical.b-p.resistance*ib-eb)/p.inductance;
      const float iq=-ia*std::sin(theta)+ib*std::cos(theta);
      w+=h*pairs*(1.5f*pairs*flux*iq-drag*w/pairs)/inertia;
      theta=wrap(theta+h*w);
    }
    const float iu=ia,iv=-.5f*ia+.8660254038f*ib,iw=-.5f*ia-.8660254038f*ib;
    peak=std::max({peak,std::abs(iu),std::abs(iv),std::abs(iw)});
    auto sensed=[](float i) { return std::round(i/.02938f)*.02938f; };
    const auto duty=c.step(cart_current(sensed(iu),sensed(iv),sensed(iw),physical_direction),19.4f,virtual_voltage);
    virtual_voltage=applied_voltage(duty,19.4f);
    physical=applied_voltage(cart_duty(duty,physical_direction),19.4f);
    if(!reversed && c.state==State::closed && first_closed<0) first_closed=n*dt;
    if(c.fault!=Fault::none) {
      printf("FAULT t=%.4f state=%s fault=%s actualRPM=%.3f estimate=%.3f\n",n*dt,name(c.state),name(c.fault),board::wheel_rpm_from_electrical_hz(w/(2*pi)),board::wheel_rpm_from_electrical_hz(c.observer.omega/(2*pi)));
      assert(false);
    }
    assert(std::abs(c.current_reference.q)<=2.0001f);
    if (c.state==State::ready && !reversed) {
      ready_at=n*dt;
      assert(std::abs(board::wheel_rpm_from_electrical_hz(w/(2*pi)))<.2f);
      assert(!c.energized());
      c.stop(); physical_direction=-direction; start(20); reversed=true;
      physical=virtual_voltage={};
    }
    if(reversed && c.state==State::closed && closed_reverse<0) closed_reverse=n*dt;
  }
  const float rpm=board::wheel_rpm_from_electrical_hz(w/(2*pi));
  printf("dir=%d theta=%.1f request_at=%.2f first_closed=%.3f brake=%.3fs reverse_closed=%.3fs final=%.3f peak=%.3fA\n",direction,initial_angle,brake_at,first_closed,ready_at-brake_at,closed_reverse-brake_at,rpm,peak);
  if(brake_at>=4) assert(first_closed>0 && first_closed<2.5f);
  assert(ready_at>brake_at && ready_at-brake_at<1);
  assert(closed_reverse>0 && closed_reverse-brake_at<3);
  assert(std::abs(rpm+direction*20)<.4f && peak<4);
  c.stop(); assert(!c.brake_speed());
}

int main() {
  setvbuf(stdout,nullptr,_IONBF,0);
  for (int direction : {1,-1}) for (float angle : {0.f,1.5f,3.f}) simulate(direction,angle);
  for (float at : {.15f,.6f,1.4f}) simulate(1,1.5f,at);
  Controller c; const Limits l{18,22,4,.2f}; const Parameters p{1.887f,.000881f};
  assert(c.run_speed(l,p,30,2,60,true,27)); assert(c.brake_speed());
  c.step({},23,{}); assert(c.fault==Fault::bus && !c.energized());
  c.stop(); assert(!c.brake_speed());
  puts("PASS: sensorless controlled stop/reverse, both directions, startup interruption, 20 RPM, quantized sensing, current/bus guards");
}
