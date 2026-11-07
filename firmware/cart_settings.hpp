#pragma once
#include <cstdint>
#include <cstring>

namespace motor {
// Wire units are integers so a readback can be checked exactly. These bounds
// define supported tuning, not a calibrated temperature or a new hardware rating.
#define CART_SETTINGS_FIELDS(X) \
  X(bus_min_mv, 18000) X(bus_max_mv, 26000) X(phase_trip_ma, 4000) \
  X(ntc_trip_raw, 2873) X(wheel_cycles_milli, 199000) \
  X(min_mrpm, 8000) X(max_mrpm, 25000) X(max_iq_ma, 2000) X(max_slew_mrpm_s, 20000) \
  X(startup_mrpm, 8000) X(startup_ma, 600) X(accel_ff_ma, 0) \
  X(sum_soft_ma, 1200) X(sum_hard_ma, 2000) X(sum_trip_samples, 10) \
  X(lock_bad_cost, 5) X(lock_score, 2000) \
  X(overspeed_filtered_milli, 1500) X(overspeed_raw_milli, 3000)

struct CartSettings {
#define FIELD(name, initial) uint32_t name = initial;
  CART_SETTINGS_FIELDS(FIELD)
#undef FIELD
  static constexpr unsigned field_count = 19;
  static const char* key(unsigned index) {
    static const char* const keys[] = {
#define KEY(name, initial) #name,
      CART_SETTINGS_FIELDS(KEY)
#undef KEY
    };
    return index < field_count ? keys[index] : nullptr;
  }
  uint32_t get(unsigned index) const {
    unsigned i=0;
#define GET(name, initial) if (index==i++) return name;
    CART_SETTINGS_FIELDS(GET)
#undef GET
    return 0;
  }
  bool set(const char* key, uint32_t value, unsigned& index) {
    index=0;
#define SET(name, initial) if (!std::strcmp(key,#name)) { name=value; return true; } ++index;
    CART_SETTINGS_FIELDS(SET)
#undef SET
    return false;
  }
  bool valid() const {
    return bus_min_mv>=7000 && bus_min_mv<bus_max_mv && bus_max_mv<=26000 &&
        phase_trip_ma>=1000 && phase_trip_ma<=4000 &&
        ntc_trip_raw>=1000 && ntc_trip_raw<=3200 &&
        wheel_cycles_milli>=1000 && wheel_cycles_milli<=1000000 &&
        min_mrpm>=8000 && min_mrpm<=max_mrpm && max_mrpm<=100000 &&
        max_iq_ma>=100 && max_iq_ma<=2000 && max_iq_ma*2<=phase_trip_ma &&
        max_slew_mrpm_s>=1000 && max_slew_mrpm_s<=30000 &&
        startup_mrpm>=8000 && startup_mrpm<=min_mrpm &&
        startup_ma>=100 && startup_ma<=1000 && startup_ma*2<=phase_trip_ma &&
        accel_ff_ma<=1000 &&
        uint64_t(startup_mrpm)*wheel_cycles_milli>=600000000ULL &&
        uint64_t(max_mrpm)*wheel_cycles_milli<=21600000000ULL &&
        uint64_t(max_slew_mrpm_s)*wheel_cycles_milli<=6000000000ULL &&
        sum_soft_ma>=300 && sum_soft_ma<=1500 && sum_hard_ma>sum_soft_ma && sum_hard_ma<=2000 &&
        sum_trip_samples>=1 && sum_trip_samples<=40 &&
        lock_bad_cost>=1 && lock_bad_cost<=100 && lock_score>=500 && lock_score<=10000 &&
        overspeed_filtered_milli>=1100 && overspeed_filtered_milli<=2000 &&
        overspeed_raw_milli>=overspeed_filtered_milli && overspeed_raw_milli<=3000;
  }
  float electrical_hz(float rpm) const { return rpm*(wheel_cycles_milli*.001f)/60; }
  float wheel_rpm(float hz) const { return hz*60/(wheel_cycles_milli*.001f); }
};
#undef CART_SETTINGS_FIELDS

// No active setting changes until every field has arrived and validates.
// A malformed/duplicate/unknown field poisons this transaction until begin().
class CartSettingsTransaction {
 public:
  CartSettings candidate{};
  bool pending=false, failed=false;
  uint32_t id=0, received=0;
  void abort() { pending=false; failed=false; id=received=0; }
  bool begin(uint32_t token) {
    abort(); candidate={};
    if (!token) return false;
    pending=true; id=token; return true;
  }
  bool put(const char* key, uint32_t value) {
    unsigned index=0;
    if (!pending || failed || !candidate.set(key,value,index) || (received & (1U<<index))) {
      failed=true; return false;
    }
    received|=1U<<index; return true;
  }
  bool ready(uint32_t token) const {
    return pending && !failed && token==id && received==((1U<<CartSettings::field_count)-1) && candidate.valid();
  }
};
}
