#ifndef POWER_MONITOR_HPP_
#define POWER_MONITOR_HPP_

#include "power_monitor_data.hpp"

class PowerMonitor {
public:
  PowerMonitor(ADCLocal &adc);
  void start(void);
  void stop(void);
  void update(power_monitor_data_t &result);
  void warmup_filters(power_monitor_data_t &result);
private:
  uint32_t adc2millivolts(adcsample_t raw_p, adcsample_t raw_n);
  main_battery_health millivolts2healt(uint32_t mv);
  bool ready = false;
  ADCLocal &adc;
  uint32_t const *cells = nullptr;
  uint32_t const *chemistry = nullptr;
  uint32_t const *ignore = nullptr;
  uint32_t const *second_v = nullptr;
  uint32_t const *main_v = nullptr;
};

#endif /* POWER_MONITOR_HPP_ */

