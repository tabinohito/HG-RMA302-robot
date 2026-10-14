#include "cart_direction.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
using namespace motor;

// An independent physical alpha/beta PMSM plant. The plant sees real phase
// voltages, not virtual-frame controller values. Reverse must produce actual
// negative angular velocity and negative physical torque with positive virtual Iq.
static float simulate(int direction) {
  const Limits limits{8,15,1,.4f};
  const Parameters parameters{1.5f,.0012f};
  Controller drive;
  assert(drive.run_current(limits,parameters,40,.3f,.5f));
  float ia=0, ib=0, theta=0, omega=0;
  AB physical_voltage{}, virtual_voltage{};
  bool closed=false;
  for (int n=0;n<160000 && drive.energized();++n) {
    const float target=n<100000 ? .01f : (n<130000 ? .04f : .05f);
    assert(drive.set_current_reference(target));
    for (int j=0;j<10;++j) {
      constexpr float h=dt/10, flux=.006f, pairs=7, inertia=.00002f, drag=.00006f;
      const float ea=-flux*omega*std::sin(theta), eb=flux*omega*std::cos(theta);
      ia+=h*(physical_voltage.a-parameters.resistance*ia-ea)/parameters.inductance;
      ib+=h*(physical_voltage.b-parameters.resistance*ib-eb)/parameters.inductance;
      const float physical_iq=-ia*std::sin(theta)+ib*std::cos(theta);
      omega+=h*pairs*(1.5f*pairs*flux*physical_iq-drag*omega/pairs)/inertia;
      theta=wrap(theta+h*omega);
    }
    // Inverse Clarke from independently integrated physical currents.
    const float iu=ia, iv=-.5f*ia+.8660254038f*ib, iw=-.5f*ia-.8660254038f*ib;
    const Duty virtual_duty=drive.step(cart_current(iu,iv,iw,direction),12,virtual_voltage);
    virtual_voltage=applied_voltage(virtual_duty,12);
    physical_voltage=applied_voltage(cart_duty(virtual_duty,direction),12);
    closed |= drive.state==State::closed;
  }
  assert(closed && drive.state==State::closed && drive.fault==Fault::none);
  assert(direction*omega>2*pi*40);
  assert(std::abs(drive.current_feedback.q-.05f)<.003f);
  assert(std::abs(wrap(direction*drive.observer.angle-theta))<.06f);
  return omega;
}

int main() {
  CartDirection direction;
  assert(!direction.begin(0,0));
  assert(direction.begin(1,0));
  assert(direction.accepts(.2f) && direction.accepts(0) && !direction.accepts(-.2f));
  assert(!direction.accepts(std::numeric_limits<float>::quiet_NaN()));
  assert(!direction.begin(-1,3000)); // active: no hot reversal
  direction.stop(3000);
  direction.stop(4000); // repeated stops do not move the original deadline
  assert(!direction.begin(-1,4999));
  assert(direction.begin(-1,5000));
  assert(direction.accepts(-.2f) && !direction.accepts(.2f));
  direction.stop(0xffffff00U);
  assert(!direction.begin(1,0x100U));
  assert(direction.begin(1,0x800U)); // unsigned tick wrap
  assert(CartDirection::start_direction(-40,-.2f)==-1);
  assert(CartDirection::start_direction(40,.2f)==1);
  assert(CartDirection::start_direction(40,-.2f)==0);
  assert(CartDirection::start_direction(40,0)==0);
  float forward=simulate(1), reverse=simulate(-1);
  assert(std::abs(forward+reverse)<.1f);
  std::printf("PASS: signed startup, hot-reverse rejection, stop dwell/tick wrap; PMSM forward %.2f Hz, reverse %.2f Hz\n",
              forward/(2*pi),reverse/(2*pi));
}
