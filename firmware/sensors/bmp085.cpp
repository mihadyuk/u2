#include <cmath>

#include "main.h"
#include "bmp085.hpp"
#include "param_registry.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

// sensor precision (see datasheet)
#define OSS 3 // 3 -- max

#define TEMP_DECIMATOR  0x1F      /* decimation value for temperature measurements */
#define PRESS_READ_TMO  MS2ST(50) /* wait pressure results (datasheet says 25.5 ms) */
#define TEMP_READ_TMO   MS2ST(10) /* wait temperature results (datasheet says 4.5 ms) */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_vfr_hud_t          mavlink_out_vfr_hud_struct;
extern mavlink_scaled_pressure_t  mavlink_out_scaled_pressure_struct;
extern mavlink_raw_pressure_t     mavlink_out_raw_pressure_struct;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/* value to calculate time between measurements and climb rate */
static systime_t measurement_time_prev;
static systime_t measurement_time;

static float altitude = 0;
static float altitude_prev = 0;
static float climb = 0;

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
static float press_to_height_f32(uint32_t pval){
  const float p0 = 101325;
  const float e = 0.1902949571836346; // 1/5.255;

  return 44330 * (1 - powf(pval/p0, e));
}

/**
 * Move pressure value to MSL as it done by meteo sites in iternet.
 */
static float press_to_msl(uint32_t pval, int32_t above_msl){
  float p = 1.0f - above_msl / 44330.0f;
  return pval / powf(p, 5.255f);
}

/**
 * Calculate compensated pressure value using black magic from datasheet.
 *
 * @ut[in]    uncompensated temperature value.
 * @ut[in]    uncompensated pressure value.
 *
 * @return    compensated pressure in Pascals.
 */
uint32_t BMP085::bmp085_calc_pressure(uint32_t ut, uint32_t up){
  // compensated temperature and pressure values
  uint32_t pval = 0;
  int32_t  tval = 0;

  int32_t  x1, x2, x3, b3, b5, b6, p;
  uint32_t  b4, b7;

  x1 = (ut - ac6) * ac5 >> 15;
  x2 = ((int32_t) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  tval = (b5 + 8) >> 4;

  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11;
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = ((((int32_t)ac1 * 4 + x3) << OSS) + 2) >> 2;

  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t)up - b3) * (50000 >> OSS);
  if(b7 < 0x80000000)
    p = (b7 * 2) / b4;
  else
    p = (b7 / b4) * 2;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357L * p) >> 16;
  pval = p + ((x1 + x2 + 3791) >> 4);

  raw_data.temp_bmp085 = (int16_t)tval;
  return pval;
}

/**
 * Calculate height and climb from pressure value
 * @pval[in]    pressure in Pascals.
 */
void BMP085::process_pressure(uint32_t pval){

  altitude = press_to_height_f32(pval);

  if (*flen_pres_stat == flen_pres_stat_cached)
    comp_data.baro_altitude = bmp085_filter.update(altitude, *flen_pres_stat);
  else{
    bmp085_filter.reset(altitude, *flen_pres_stat);
    flen_pres_stat_cached = *flen_pres_stat;
  }

  measurement_time = chTimeNow();
  climb = (altitude_prev - comp_data.baro_altitude) *
          (float)(CH_FREQUENCY / (measurement_time - measurement_time_prev));
  measurement_time_prev = measurement_time;
  altitude_prev = comp_data.baro_altitude;

  comp_data.baro_climb = bmp085_climb_filter.update(climb, *flen_climb);

  mavlink_out_vfr_hud_struct.alt = comp_data.baro_altitude;
  mavlink_out_vfr_hud_struct.climb = comp_data.baro_climb;
  mavlink_out_scaled_pressure_struct.press_abs = press_to_msl(pval, *above_msl) / 100.0f;
  mavlink_out_raw_pressure_struct.press_abs = pval / 100;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
BMP085::BMP085(I2CDriver* i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr)
{
  measure = MEASURE_NONE;
  up  = 0; ut  = 0;
  ac1 = 0; ac2 = 0; ac3 = 0; b1 = 0; b2 = 0; mb = 0; mc = 0; md = 0;
  ac4 = 0; ac5 = 0; ac6 = 0;
  flen_pres_stat_cached = 1;
  ready = false;
}

/**
 *
 */
void BMP085::update(void){

  osalDbgCheck(ready);

  switch(measure){
  case MEASURE_NONE:
    /* start temerature measurement */
    txbuf[0] = BOSCH_CTL;
    txbuf[1] = BOSCH_TEMP;
    transmit(txbuf, 2, rxbuf, 0);
    measure = MEASURE_T;
    break;

  case MEASURE_T:
    /* acquire temperature measurement */
    txbuf[0] = BOSCH_ADC_MSB;
    transmit(txbuf, 1, rxbuf, 2);
    ut = (rxbuf[0] << 8) + rxbuf[1];

    /* fire up pressure measurement */
    txbuf[0] = BOSCH_CTL;
    txbuf[1] = (0x34 + (OSS<<6));
    transmit(txbuf, 2, rxbuf, 0);
    measure = MEASURE_P;
    break;

  case MEASURE_P:
    /* acqure pressure value */
    txbuf[0] = BOSCH_ADC_MSB;
    transmit(txbuf, 1, rxbuf, 3);
    up = ((rxbuf[0] << 16) + (rxbuf[1] << 8) + rxbuf[2]) >> (8 - OSS);

    /* start temerature measurement */
    txbuf[0] = BOSCH_CTL;
    txbuf[1] = BOSCH_TEMP;
    transmit(txbuf, 2, rxbuf, 0);
    measure = MEASURE_T;
    break;

  default:
    osalSysHalt("unhanlded case");
    break;
  }

  this->pickle();
}

/**
 *
 */
msg_t BMP085::start(void) {
  if (need_full_init())
    hw_init_full();
  else
    hw_init_fast();

  param_registry.valueSearch("FLEN_pres_stat", &flen_pres_stat);
  param_registry.valueSearch("FLEN_climb", &flen_climb);
  param_registry.valueSearch("PMU_above_msl", &above_msl);

  ready = true;

  return RDY_OK;
}

/**
 *
 */
void BMP085::stop(void) {
  measure = MEASURE_NONE;
  ready = false;
}

/**
 *
 */
void BMP085::pickle(void) {
  raw_data.pressure_static = bmp085_calc_pressure(ut, up);
  process_pressure(raw_data.pressure_static);
}

/**
 *
 */
msg_t BMP085::hw_init_full(void) {
  /* get calibration coefficients from sensor */
  txbuf[0] = 0xAA;
  transmit(txbuf, 1, rxbuf, 22);

  ac1 = (rxbuf[0]  << 8) + rxbuf[1];
  ac2 = (rxbuf[2]  << 8) + rxbuf[3];
  ac3 = (rxbuf[4]  << 8) + rxbuf[5];
  ac4 = (rxbuf[6]  << 8) + rxbuf[7];
  ac5 = (rxbuf[8]  << 8) + rxbuf[9];
  ac6 = (rxbuf[10] << 8) + rxbuf[11];
  b1  = (rxbuf[12] << 8) + rxbuf[13];
  b2  = (rxbuf[14] << 8) + rxbuf[15];
  mb  = (rxbuf[16] << 8) + rxbuf[17];
  mc  = (rxbuf[18] << 8) + rxbuf[19];
  md  = (rxbuf[20] << 8) + rxbuf[21];

  return RDY_OK;
}

/**
 *
 */
msg_t BMP085::hw_init_fast(void) {
  return this->hw_init_full();
}

/**
 *
 */
bool TrigCalibrateBaro(){
  return CH_FAILED;
}
