#ifndef POWER_MONITOR_DATA_HPP_
#define POWER_MONITOR_DATA_HPP_

/**
 *
 */
enum class main_battery_health {
  GOOD,
  LOW,
  CRITICAL
};

/**
 *
 */
struct power_monitor_data_t {
  float main_voltage;       // V
  float secondary_voltage;  // V
  float main_current;       // A
  float consumed_power;     // Ah
  main_battery_health health;
};

#endif /* POWER_MONITOR_DATA_HPP_ */
