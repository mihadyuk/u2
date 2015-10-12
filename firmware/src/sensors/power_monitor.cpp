#include "main.h"
#include "power_monitor.hpp"

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

//#define LIPO_LOW                3400 // mV
//#define LIPO_CRITICAL           3250 // mV
//
//#define LEAD_ACID_LOW           1920 // mV
//#define LEAD_ACID_CRITICAL      1750 // mV

#define MAIN_VOLTAGE_IGNORE     400  // sensor not connected at all

/**
 * @brief   Battery chemistry supported types
 */
typedef enum {
  BAT_CHEMISTRY_LIPO = 0,
  BAT_CHEMISTRY_LEAD_ACID,
  BAT_CHEMISTRY_ENUM_END,
} bat_chemistry_t;

/**
 *
 */
struct constrain {
  constrain(uint32_t low, uint32_t critical) : low(low), critical(critical) {;}
  const uint32_t low;
  const uint32_t critical;
};

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static const constrain voltage_constrains[BAT_CHEMISTRY_ENUM_END] = {
    (3400, 3250),   // LiPo
    (1920, 1750)    // Lead acid
};

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/*
 * пересчет из условных единиц АЦП в mV
 */
uint16_t PowerMonitor::comp_secondary_voltage(uint16_t raw) {
  uint32_t uV = raw;

  uV *= *adc_sv_gain;
  return uV / 1000;
}

/*
 * пересчет из условных единиц АЦП в mV
 */
uint16_t PowerMonitor::comp_main_voltage(uint16_t raw) {
  uint32_t uV = raw;

  uV *= *adc_mv_gain;
  return uV / 1000;
}

/**
 *
 */
main_battery_health PowerMonitor::translate_voltage(uint16_t mv) {

  uint32_t tmp = (uint32_t)mv / *cells;

  osalDbgCheck(*chemistry < BAT_CHEMISTRY_ENUM_END);

  if (voltage_constrains[*chemistry].critical < tmp)
    return main_battery_health::CRITICAL;
  else if (voltage_constrains[*chemistry].low < tmp)
    return main_battery_health::LOW;
  else
    return main_battery_health::GOOD;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
void PowerMonitor::start(void) {
  param_registry.valueSearch("BAT_cells",     &cells);
  param_registry.valueSearch("BAT_chemistry", &chemistry);
  param_registry.valueSearch("ADC_SV_gain",   &adc_sv_gain);
  param_registry.valueSearch("ADC_MV_gain",   &adc_mv_gain);

  ready = true;
}

/**
 *
 */
void PowerMonitor::stop(void) {

  ready = false;
}

/**
 *
 */
void PowerMonitor::update(power_monitor_data_t &result) {
  osalDbgCheck(ready);

  result.health = translate_voltage(comp_main_voltage(ADCgetMainVoltage()));
}




