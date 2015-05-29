#pragma GCC optimize "-O2"

#include <cmath>

#include "main.h"
#include "mavlink_local.hpp"
#include "bmp085.hpp"
#include "param_registry.hpp"
#include "exti_local.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

// sensor precision (see datasheet)
#define OSS 3 // 3 -- max

#define BOSCH_CTL           0xF4 // control register address
#define BOSCH_TEMP          0x2E
#define BOSCH_PRES          0xF4 // pressure with OSRS=3 (page 17 in manual)
#define BOSCH_ADC_MSB       0xF6
#define BOSCH_ADC_LSB       0xF7
#define BOSCH_ADC_XLSB      0xF8
#define BOSCH_TYPE          0xD0

#define PRESSURE_CONVERSION_TIME_MS        27
#define TEMPERATURE_CONVERSION_TIME_MS     6

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_scaled_pressure_t  mavlink_out_scaled_pressure_struct;
extern mavlink_raw_pressure_t     mavlink_out_raw_pressure_struct;
extern mavlink_vfr_hud_t          mavlink_out_vfr_hud_struct;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static chibios_rt::BinarySemaphore isr_sem(true);

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
 * Calculate compensated pressure value using black magic from datasheet.
 *
 * @ut[in]    uncompensated temperature value.
 * @up[in]    uncompensated pressure value.
 *
 * @return    compensated pressure in Pascals.
 */
void BMP085::calc_pressure(void) {
  int32_t  tval;
  int32_t  x1, x2, x3, b3, b5, b6, p;
  uint32_t  b4, b7;

  x1 = (ut - ac6) * ac5 >> 15;
  x2 = ((int32_t) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  tval = (b5 + 8) >> 4;
  (void)tval;

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
  pressure_compensated = p + ((x1 + x2 + 3791) >> 4);
}

/**
 *
 */
bool BMP085::start_t_measurement(void) {

  txbuf[0] = BOSCH_CTL;
  txbuf[1] = BOSCH_TEMP;
  if (MSG_OK != transmit(txbuf, 2, rxbuf, 0))
    return OSAL_FAILED;
  else
    return OSAL_SUCCESS;
}

/**
 *
 */
bool BMP085::start_p_measurement(void) {
  txbuf[0] = BOSCH_CTL;
  txbuf[1] = (0x34 + (OSS<<6));
  if (MSG_OK != transmit(txbuf, 2, rxbuf, 0))
    return OSAL_FAILED;
  else
    return OSAL_SUCCESS;
}

/**
 *
 */
bool BMP085::acquire_t(void) {
  txbuf[0] = BOSCH_ADC_MSB;
  if (MSG_OK != transmit(txbuf, 1, rxbuf, 2))
    return OSAL_FAILED;
  else {
    ut = (rxbuf[0] << 8) + rxbuf[1];
    return OSAL_SUCCESS;
  }
}

/**
 *
 */
bool BMP085::acquire_p(void) {
  txbuf[0] = BOSCH_ADC_MSB;
  if (MSG_OK != transmit(txbuf, 1, rxbuf, 3))
    return OSAL_FAILED;
  else {
    up = ((rxbuf[0] << 16) + (rxbuf[1] << 8) + rxbuf[2]) >> (8 - OSS);
    return OSAL_SUCCESS;
  }
}

/**
 * Calculate height and climb from pressure value
 * @pval[in]    pressure in Pascals.
 */
void BMP085::picle(baro_data_t &result) {

  float altitude = press2height(pressure_compensated);

  result.alt = altitude_filter(altitude, *flen_pres_stat);
  result.climb = climb(result.alt);
  result.p = pressure_compensated;
  result.p_msl_adjusted = press2msl(pressure_compensated, *above_msl);

  mavlink_out_scaled_pressure_struct.press_abs = pressure_compensated / 100.0f;
  mavlink_out_raw_pressure_struct.press_abs = pressure_compensated / 100;
}

/**
 *
 */
void BMP085::baro2mavlink(void) {

  mavlink_out_vfr_hud_struct.alt = cache.alt;
  mavlink_out_vfr_hud_struct.climb = cache.climb;
}

/**
 *
 */
__CCM__ static THD_WORKING_AREA(bmp085ThreadWA, 256);
THD_FUNCTION(bmp085Thread, arg) {
  chRegSetThreadName("bmp085");
  BMP085 *sensor = (BMP085 *)arg;
  baro_data_t result;
  systime_t starttime;

  while (!chThdShouldTerminateX()) {
    if (SENSOR_STATE_READY == sensor->state) {
      starttime = chVTGetSystemTime();
      sensor->acquire_p();
      sensor->start_t_measurement();
      chThdSleepUntilWindowed(starttime, starttime + MS2ST(TEMPERATURE_CONVERSION_TIME_MS));

      starttime += MS2ST(TEMPERATURE_CONVERSION_TIME_MS);
      sensor->acquire_t();
      sensor->start_p_measurement();
      chThdSleepUntilWindowed(starttime, starttime + MS2ST(PRESSURE_CONVERSION_TIME_MS));

      sensor->calc_pressure();
      sensor->picle(result);
      osalSysLock();
      sensor->cache = result;
      osalSysUnlock();

      sensor->baro2mavlink();
    }
    else {
      osalThreadSleepMilliseconds(TEMPERATURE_CONVERSION_TIME_MS + PRESSURE_CONVERSION_TIME_MS);
    }
  }

  chThdExit(MSG_OK);
}

/**
 *
 */
float BMP085::climb(float alt) {

  float dt, climb;

  dt = TEMPERATURE_CONVERSION_TIME_MS + PRESSURE_CONVERSION_TIME_MS;
  dt /= 1000;
  climb = climb_filter((alt - altitude_prev) / dt, *flen_climb);
  altitude_prev = alt;

  return climb;
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
//measure_type(MEASURE_NONE),
//up(0),  ut(0),
//ac1(0), ac2(0), ac3(0), b1(0), b2(0), mb(0), mc(0), md(0),
//ac4(0), ac5(0), ac6(0)
{
  state = SENSOR_STATE_STOP;
}

/**
 *
 */
void BMP085::stop(void) {
  if (this->state == SENSOR_STATE_STOP)
    return;

  osalDbgAssert((this->state == SENSOR_STATE_READY) ||
                (this->state == SENSOR_STATE_SLEEP), "Invalid state");
  this->state = SENSOR_STATE_STOP;
  chThdTerminate(worker);
  chThdWait(worker);
  Exti.bmp085(false);
}

/**
 *
 */
void BMP085::sleep(void) {
  if (this->state == SENSOR_STATE_SLEEP)
    return;

  osalDbgAssert(this->state == SENSOR_STATE_READY, "Invalid state");
  this->state = SENSOR_STATE_SLEEP;
}

/**
 *
 */
sensor_state_t BMP085::get(baro_data_t &result) {

  result = cache;
  return this->state;
}

/**
 *
 */
sensor_state_t BMP085::wakeup(void) {
  if (this->state == SENSOR_STATE_READY)
    return this->state;

  osalDbgAssert(this->state == SENSOR_STATE_SLEEP, "Invalid state");
  this->state = SENSOR_STATE_READY;
  return this->state;
}

/**
 *
 */
bool BMP085::hw_init_full(void) {
  msg_t ret = MSG_RESET;

  /* get calibration coefficients from sensor */
  txbuf[0] = 0xAA;
  ret = transmit(txbuf, 1, rxbuf, 22);

  if (MSG_OK != ret) // does not responding
    return OSAL_FAILED;
  else {
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
    return OSAL_SUCCESS;
  }
}

/**
 *
 */
bool BMP085::hw_init_fast(void) {
  return this->hw_init_full();
}

/**
 *
 */
sensor_state_t BMP085::start(void) {
  bool init_status = OSAL_FAILED;

  if (SENSOR_STATE_STOP == this->state) {
    if (need_full_init())
      init_status = hw_init_full();
    else
      init_status = hw_init_fast();

    /* check state */
    if (OSAL_SUCCESS != init_status) {
      this->state = SENSOR_STATE_DEAD;
      return this->state;
    }
    else {
      param_registry.valueSearch("FLEN_pres_stat",  &flen_pres_stat);
      param_registry.valueSearch("FLEN_climb",      &flen_climb);
      param_registry.valueSearch("PMU_above_msl",   &above_msl);
      worker = chThdCreateStatic(bmp085ThreadWA, sizeof(bmp085ThreadWA),
                                 BAROMETERPRIO, bmp085Thread, this);
      osalDbgCheck(nullptr != worker);
      Exti.bmp085(true);
      this->state = SENSOR_STATE_READY;
    }
  }

  return this->state;
}

/**
 *
 */
void BMP085::extiISR(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  osalSysLockFromISR();
  isr_sem.signalI();
  osalSysUnlockFromISR();
}

