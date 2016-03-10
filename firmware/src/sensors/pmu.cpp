#include <cmath>
#include <cstdlib>

#include "main.h"
#include "mavlink_local.hpp"
#include "pmu.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
//extern mavlink_raw_pressure_t     mavlink_out_raw_pressure_struct;
//extern mavlink_scaled_pressure_t  mavlink_out_scaled_pressure_struct;
extern mavlink_highres_imu_t      mavlink_out_highres_imu_struct;
extern mavlink_vfr_hud_t          mavlink_out_vfr_hud_struct;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
/**
 * Calculate height in meters using proved formulae
 */
static double press2height(uint32_t pval) {
  const double p0 = 101325;
  const double e = 0.1902949571836346; /* 1/5.255 */

  return 44330 * (1 - pow(pval/p0, e));
}

/**
 * Move pressure value to MSL as it done by meteo sites in iternet.
 */
static double press2msl(uint32_t pval, int32_t height) {
  double p = 1.0 - height / 44330.0;
  return pval / pow(p, 5.255);
}

/**
 *
 */
static float press2airspeed(int32_t press_diff){

  float p = abs(press_diff);
  float v = sqrtf((2*p) / 1.225f);

  if (press_diff < 0)
    return -v;
  else
    return v;
}

/**
 *
 */
static void baro2mavlink(const baro_data_t &data,
                         const baro_diff_data_t &diff,
                         const baro_abs_data_t &abs) {
  /* TODO: remove unused parameters */

  /* mute warning: unused parameter */
  (void)diff;
  (void)abs;

  mavlink_out_vfr_hud_struct.alt = data.alt;
  mavlink_out_vfr_hud_struct.climb = data.climb;
  mavlink_out_vfr_hud_struct.airspeed = data.airspeed;

  // Fill baro data only in HIGHRES_IMU mavlink package
  mavlink_out_highres_imu_struct.abs_pressure = data.p_abs;
  mavlink_out_highres_imu_struct.diff_pressure = data.p_diff;
  mavlink_out_highres_imu_struct.pressure_alt = data.alt;
  mavlink_out_highres_imu_struct.temperature = data.t;
  /*
  mavlink_out_scaled_pressure_struct.press_abs = data.p_abs;
  mavlink_out_scaled_pressure_struct.press_diff = data.p_diff;
  mavlink_out_scaled_pressure_struct.time_boot_ms = TIME_BOOT_MS;

  mavlink_out_raw_pressure_struct.press_diff1 = diff.raw;
  osalSysLock();
  mavlink_out_raw_pressure_struct.press_abs    = 0;
  mavlink_out_raw_pressure_struct.press_abs   |= (abs.p_raw >> 16) & 0xFFFF;
  mavlink_out_raw_pressure_struct.press_diff2  = 0;
  mavlink_out_raw_pressure_struct.press_diff2 |= abs.p_raw & 0xFFFF;
  mavlink_out_raw_pressure_struct.temperature  = 0;
  mavlink_out_raw_pressure_struct.temperature |= (abs.t_raw >> 8) & 0xFFFF;
  osalSysUnlock();
  */
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
void PMUGet(const baro_abs_data_t &abs, const baro_diff_data_t &diff,
            const float gnss_alt, baro_data_t &result) {

  result.alt = press2height(abs.p);
  result.p_abs = abs.p;
  result.p_msl_adjusted = press2msl(abs.p, gnss_alt);
  result.t = abs.t;
  result.climb = 0;

  result.p_diff = diff.p;
  result.airspeed = press2airspeed(diff.p);

  baro2mavlink(result, diff, abs);
}




