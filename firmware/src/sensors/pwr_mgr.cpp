#include <cmath>

#include "main.h"

#include "mavlink_local.hpp"
#include "adc_local.hpp"
#include "param_registry.hpp"
#include "alpha_beta.hpp"

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern mavlink_sys_status_t   mavlink_out_sys_status_struct;
extern mavlink_system_info_t  mavlink_system_info_struct;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define SECONDARY_VOLTAGE_GOOD    5400 // mV

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
static uint32_t const *bat_cap;       /* battery capacitance in A*mS */
static uint32_t const *bat_fill;      /* battery filling in A*mS */
static int32_t  const *adc_I_b;
static int32_t  const *adc_I_k;
static uint32_t const *adc_sv_gain;   /* secondary voltage gain */
static uint32_t const *adc_mv_gain;   /* main voltage gain */

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
static uint16_t comp_secondary_voltage(uint16_t raw) {
  uint32_t uV = raw;

  uV *= *adc_sv_gain;
  return uV / 1000;
}

/*
 * пересчет из условных единиц АЦП в mV
 */
static uint16_t comp_main_voltage(uint16_t raw) {
  uint32_t uV = raw;

  uV *= *adc_mv_gain;
  return uV / 1000;
}

/* STUB */
//static uint16_t get_comp_main_voltage(uint16_t raw){
//  return (uint16_t)(((uint32_t)raw * *adc_mv_gain) / 1000);
//}

/* пересчет из условных единиц в mA согласно формуле y=kx+b
 * Формула выводится из уравнения прямой
 *
 * y - y1    x - x1
 * ------- = -------
 * y2 - y1   x2 - x1
 *
 *     (x - x1) * (y2 - y1)
 * y = -------------------  + y1
 *            x2 - x1
 *
 * Для того, чтобы воспользоваться ей, нам необходимо снять 2 точки.
 * Для машинки:
 * (АЦП; мА) -- (193;87), (388;200)
 * откуда получается (мА):
 * y = 0.58x + 24.841
 * или (мкА)
 * y = 580x + 24841
 */
static uint32_t get_comp_main_current(uint16_t raw){
  return ((((uint32_t)raw) * *adc_I_k) + *adc_I_b) / 1000;
}

/*
 * Process ADC data.
 */
void PwrMgrUpdate(void) {
  uint32_t main_current = get_comp_main_current(ADCgetCurrent());
  (void)main_current;

  mavlink_out_sys_status_struct.current_battery = -1;
  mavlink_out_sys_status_struct.voltage_battery = comp_main_voltage(ADCgetMainVoltage());
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
void PwrMgrInit(void){
  param_registry.valueSearch("BAT_cap", &bat_cap);
  param_registry.valueSearch("BAT_fill", &bat_fill);

  if (mavlink_system_info_struct.type == MAV_TYPE_GROUND_ROVER){
    param_registry.valueSearch("ADC_car_I_k", &adc_I_k);
    param_registry.valueSearch("ADC_car_I_b", &adc_I_b);
  }
  else{
    param_registry.valueSearch("ADC_plane_I_k", &adc_I_k);
    param_registry.valueSearch("ADC_plane_I_b", &adc_I_b);
  }

  param_registry.valueSearch("ADC_SV_gain", &adc_sv_gain);
  param_registry.valueSearch("ADC_MV_gain", &adc_mv_gain);
}

/**
 *
 */
bool PwrMgr6vGood(void) {
  const int32_t len = 16;
  int32_t tmp;
  filters::AlphaBeta<int32_t, len> voltage_filter(ADCget6v());

  for (size_t i=0; i<len*2; i++) {
    tmp = voltage_filter(ADCget6v());
    osalThreadSleepMilliseconds(1);
  }

  return comp_secondary_voltage(tmp) >= SECONDARY_VOLTAGE_GOOD;
}



