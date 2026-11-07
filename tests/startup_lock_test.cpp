#include "startup_lock.hpp"
#include <cassert>
#include <cstdio>

int main() {
  using motor::startup_lock_score;
  unsigned score = 0;
  for (unsigned n=0; n<1999; ++n) score=startup_lock_score(score,true,true);
  assert(score==1999);
  score=startup_lock_score(score,true,true); assert(score==2000);
  // 87.5% quality covers the measured ~11% brief observer noise at 6S voltage.
  score=0;
  for (unsigned n=0; n<10000; ++n) score=startup_lock_score(score,n%8!=0,true);
  assert(score==2000);
  // 80% quality, continuous invalid data and gross geometry errors cannot lock.
  score=0;
  for (unsigned n=0; n<120000; ++n) {
    score=startup_lock_score(score,n%5!=0,true); assert(score<2000);
  }
  assert(startup_lock_score(1999,true,false)==0);
  score=2000;
  for (unsigned n=0; n<400; ++n) score=startup_lock_score(score,false,true);
  assert(score==0);
  assert(startup_lock_score(4,false,true)==0);
  puts("PASS: clean acquisition, isolated observer noise, invalid observation/geometry rejection");
}
