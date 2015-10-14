#include <cmath>

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
extern mavlink_scaled_pressure_t  mavlink_out_scaled_pressure_struct;
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
static void baro2mavlink(const baro_data_t &data) {

  mavlink_out_vfr_hud_struct.alt = data.alt;
  mavlink_out_vfr_hud_struct.climb = data.climb;
  mavlink_out_vfr_hud_struct.airspeed = data.airspeed;

  mavlink_out_scaled_pressure_struct.press_abs = data.p_abs;
  mavlink_out_scaled_pressure_struct.press_diff = data.p_diff;
  mavlink_out_scaled_pressure_struct.time_boot_ms = TIME_BOOT_MS;
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

  result.alt = press2height(abs.P);
  result.p_abs = abs.P;
  result.p_msl_adjusted = press2msl(abs.P, gnss_alt);
  result.climb = 0;

  result.p_diff = diff.P;
  result.airspeed = 0;

  baro2mavlink(result);
}




