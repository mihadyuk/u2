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
  float main_voltage;         // V
  float second_voltage;       // V
  float main_current;         // A
  float second_current;       // A
  float consumed_power;       // Ah
  float cell_voltage[8];      // V
  main_battery_health health;
  uint32_t presence_vector;
};

#endif /* POWER_MONITOR_DATA_HPP_ */
