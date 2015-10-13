#include <cstring>

#include "main.h"

#include "ms5806.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define CMD_RESET       0x1E // ADC reset command
#define CMD_ADC_CONV    0x40 // ADC conversion command
#define CMD_ADC_READ    0x00 // ADC read command
#define CMD_ADC_D1      0x00 // ADC D1 conversion
#define CMD_ADC_D2      0x10 // ADC D2 conversion
#define CMD_ADC_256     0x00 // ADC OSR=256
#define CMD_ADC_512     0x02 // ADC OSR=512
#define CMD_ADC_1024    0x04 // ADC OSR=1024
#define CMD_ADC_2048    0x06 // ADC OSR=2048
#define CMD_ADC_4096    0x08 // ADC OSR=4096
#define CMD_PROM_RD     0xA0 // Prom read command

#define PRESS_CONV_TIME_MS      10
#define TEMP_CONV_TIME_MS       10

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

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
 * Calculate compensated pressure value using black magic from datasheet.
 *
 * @ut[in]    uncompensated temperature value.
 * @up[in]    uncompensated pressure value.
 *
 * @return    compensated pressure in Pascals.
 */
void MS5806::calc_pressure(void) {

}

/**
 *
 */
void MS5806::pickle(baro_data_t &result) {

 (void)result;
}

/**
 *
 */
bool MS5806::start_t_measurement(void) {

  uint8_t cmd = CMD_ADC_CONV | CMD_ADC_D2 | CMD_ADC_4096;

  if (MSG_OK != transmit(&cmd, 1, nullptr, 0))
    return OSAL_FAILED;
  else
    return OSAL_SUCCESS;
}

/**
 *
 */
bool MS5806::start_p_measurement(void) {

  uint8_t cmd = CMD_ADC_CONV | CMD_ADC_D1 | CMD_ADC_4096;

  if (MSG_OK != transmit(&cmd, 1, nullptr, 0))
    return OSAL_FAILED;
  else
    return OSAL_SUCCESS;
}

/**
 *
 */
bool MS5806::acquire_data(uint8_t *rxbuf) {

  uint8_t cmd = CMD_ADC_READ;

  if (MSG_OK != transmit(&cmd, 1, rxbuf, 3))
    return OSAL_FAILED;
  else
    return OSAL_SUCCESS;
}

/**
 *
 */
bool MS5806::hw_init_full(void) {
  uint8_t cmd;
  msg_t status;

  for (size_t i=0; i<MS5806_CAL_WORDS; i++) {
    cmd = CMD_PROM_RD;
    cmd |= i << 1;
    status = transmit(&cmd, 1, (uint8_t*)&C[i], 2);
    osalDbgCheck(MSG_OK == status);
  }

  return OSAL_SUCCESS;
}

/**
 *
 */
bool MS5806::hw_init_fast(void){
  return OSAL_SUCCESS;
}

/**
 *
 */
static THD_WORKING_AREA(ms5806ThreadWA, 256);
THD_FUNCTION(ms5806Thread, arg) {
  chRegSetThreadName("ms5806");
  MS5806 *sensor = (MS5806 *)arg;
  systime_t starttime;
  baro_data_t result;

  while (!chThdShouldTerminateX()) {
    if (SENSOR_STATE_READY == sensor->state) {
      starttime = chVTGetSystemTime();
      sensor->acquire_data(sensor->rxbuf_p);
      sensor->start_t_measurement();
      chThdSleepUntilWindowed(starttime, starttime + MS2ST(TEMP_CONV_TIME_MS));

      starttime += MS2ST(TEMP_CONV_TIME_MS);
      sensor->acquire_data(sensor->rxbuf_t);
      sensor->start_p_measurement();
      chThdSleepUntilWindowed(starttime, starttime + MS2ST(PRESS_CONV_TIME_MS));

      sensor->calc_pressure();
      sensor->pickle(result);
      osalSysLock();
      sensor->cache = result;
      osalSysUnlock();
      osalThreadSleepMilliseconds(TEMP_CONV_TIME_MS + PRESS_CONV_TIME_MS);

      //sensor->baro2mavlink();
    }
    else {
      osalThreadSleepMilliseconds(TEMP_CONV_TIME_MS + PRESS_CONV_TIME_MS);
    }
  }

  chThdExit(MSG_OK);
}



/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
MS5806::MS5806(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr),
worker(nullptr)
{
  memset(this->C, 0, sizeof(C));
  state = SENSOR_STATE_STOP;
}

/**
 *
 */
sensor_state_t MS5806::start(void) {
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
      worker = chThdCreateStatic(ms5806ThreadWA, sizeof(ms5806ThreadWA),
                                 BAROMETERPRIO, ms5806Thread, this);
      osalDbgCheck(nullptr != worker);
      this->state = SENSOR_STATE_READY;
    }
  }

  return this->state;
}

/**
 *
 */
void MS5806::stop(void){
  if (this->state == SENSOR_STATE_STOP)
    return;

  osalDbgAssert((this->state == SENSOR_STATE_READY) ||
                (this->state == SENSOR_STATE_SLEEP), "Invalid state");
  this->state = SENSOR_STATE_STOP;
  chThdTerminate(worker);
  chThdWait(worker);
}

/**
 *
 */
sensor_state_t MS5806::wakeup(void) {
  state = SENSOR_STATE_READY;
  return state;
}

/**
 *
 */
void MS5806::sleep(void) {
  state = SENSOR_STATE_SLEEP;
}

/**
 *
 */
sensor_state_t MS5806::get(baro_data_t &result) {
  osalDbgCheck(state == SENSOR_STATE_READY);

  result = cache;

  return this->state;
}
