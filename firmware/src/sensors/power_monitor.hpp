#ifndef POWER_MONITOR_HPP_
#define POWER_MONITOR_HPP_

#include "power_monitor_data.hpp"

class PowerMonitor {
public:
  void PowerMonitor(void);
  void start(void);
  void stop(void);
  void update(power_monitor_data_t &result);
private:
  main_battery_health translate_voltage(uint16_t mv);
  uint16_t comp_main_voltage(uint16_t raw);
  uint16_t comp_secondary_voltage(uint16_t raw);
  bool ready = false;
  uint32_t const *cells, *chemistry;
  uint32_t const *adc_sv_gain, *adc_mv_gain;
};

#endif /* POWER_MONITOR_HPP_ */

