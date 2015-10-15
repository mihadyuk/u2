#include "main.h"

#include "mavlink_local.hpp"
#include "adc_local.hpp"
#include "power_monitor.hpp"
#include "param_registry.hpp"

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern mavlink_sys_status_t   mavlink_out_sys_status_struct;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#if defined(BOARD_BEZVODIATEL)
  #define ADC_MPXV_PRESS        ADC_CHANNEL_IN10
  #define ADC_MPXV_TEMP         ADC_CHANNEL_IN11
  #define ADC_MAIN_CURRENT      ADC_CHANNEL_IN12
  #define ADC_MAIN_VOLTAGE      ADC_CHANNEL_IN13
  #define ADC_6V_VOLTAGE        ADC_CHANNEL_IN14
  #define ADC_RESERVED          ADC_CHANNEL_IN15
#elif defined(BOARD_MNU)
  #define ADC_MAIN_VOLTAGE_P    ADC_CHANNEL_IN10
  #define ADC_MAIN_VOLTAGE_N    ADC_CHANNEL_IN11
#else
#error "board unsupported"
#endif

//#define LIPO_LOW                3400 // mV
//#define LIPO_CRITICAL           3200 // mV
//
//#define LEAD_ACID_LOW           1920 // mV
//#define LEAD_ACID_CRITICAL      1750 // mV

#define MAIN_VOLTAGE_IGNORE       1000  // mV, sensor not connected at all

/**
 * @brief   Battery chemistry supported types
 */
typedef enum {
  BAT_CHEMISTRY_LIPO = 0,
  BAT_CHEMISTRY_LEAD_ACID,
  BAT_CHEMISTRY_ENUM_END,
} bat_chemistry_t;

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
/*
 * voltage constrains (mV per cell)
 */
static const uint32_t voltage_constrains[BAT_CHEMISTRY_ENUM_END][2] = {
    {3400, 3200},   // LiPo
    {1920, 1750}    // Lead acid
};

static filters::AlphaBeta<int32_t, 128> main_voltage_p_filter;
#if defined(BOARD_MNU)
static filters::AlphaBeta<int32_t, 128> main_voltage_n_filter;
#endif

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/**
 * return mV
 */
uint32_t PowerMonitor::adc2millivolts(adcsample_t raw_p, adcsample_t raw_n) {

  uint32_t uV;

  if (raw_p < raw_n) {
    return 0;
  }
  else {
    uV = *main_v * (raw_p - raw_n);
    return uV / 1000;
  }
}

/**
 *
 */
main_battery_health PowerMonitor::millivolts2healt(uint32_t mv) {

  uint32_t cell_mv = mv / *cells;

  osalDbgCheck(*chemistry < BAT_CHEMISTRY_ENUM_END);

  if ((mv < MAIN_VOLTAGE_IGNORE) || (1 == *ignore)) {
    return main_battery_health::GOOD;
  }
  else {
    if (cell_mv < voltage_constrains[*chemistry][1])
      return main_battery_health::CRITICAL;
    else if (cell_mv < voltage_constrains[*chemistry][0])
      return main_battery_health::LOW;
    else
      return main_battery_health::GOOD;
  }
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
PowerMonitor::PowerMonitor(ADCLocal &adc) :
adc(adc)
{
  return;
}

/**
 *
 */
void PowerMonitor::start(void) {
  param_registry.valueSearch("BAT_cells",     &cells);
  param_registry.valueSearch("BAT_chemistry", &chemistry);
  param_registry.valueSearch("BAT_ignore",    &ignore);
  param_registry.valueSearch("ADC_second_v",  &second_v);
  param_registry.valueSearch("ADC_main_v",    &main_v);

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
  uint32_t mV;
  adcsample_t p;
  adcsample_t n;

  osalDbgCheck(ready);

#if defined(BOARD_BEZVODIATEL)
  p = adc.getChannel(ADC_MAIN_VOLTAGE, main_voltage_p_filter);
  n = 0;
#elif defined(BOARD_MNU)
  p = adc.getChannel(ADC_MAIN_VOLTAGE_P, main_voltage_p_filter);
  n = adc.getChannel(ADC_MAIN_VOLTAGE_N, main_voltage_n_filter);
#else
#error "board unsupported"
#endif

  mV = adc2millivolts(p, n);

  result.health = millivolts2healt(mV);
  result.main_voltage = mV / 1000.0f;

  mavlink_out_sys_status_struct.current_battery = -1;
  mavlink_out_sys_status_struct.voltage_battery = mV;
}

/**
 * @brief   Warm filters up for false positive avoidance.
 * @note    Used only during start up procedure.
 */
void PowerMonitor::warmup_filters(power_monitor_data_t &result) {

  for (size_t i=0; i<256; i++) {
    this->update(result);
  }
}



