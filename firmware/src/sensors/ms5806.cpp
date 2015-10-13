#include <cstring>
#include <cmath>

#include "main.h"
#include "pack_unpack.h"
#include "mavlink_local.hpp"

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

#define PRESS_CONV_TIME      MS2ST(10)
#define TEMP_CONV_TIME       MS2ST(10)

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
 * Calculate compensated pressure value using black magic from datasheet.
 */
void MS5806::calc_pressure(baro_abs_data_t &result) {
  int64_t D1; // P
  int64_t D2; // T

  int64_t dT;
  int64_t temp;
  int64_t off;
  int64_t sens;
  int64_t P;


  D1 = rxbuf_p[0] * 65536 + rxbuf_p[1] * 256 + rxbuf_p[2];
  D2 = rxbuf_t[0] * 65536 + rxbuf_t[1] * 256 + rxbuf_t[2];

  dT = D2 - (int32_t)C[5] * (1<<8);
  temp = 2000 + dT*C[6] / (1<<23);

  off  = (int64_t)C[2] * (1<<17) + C[4]*dT / (1<<6);
  sens = (int64_t)C[1] * (1<<16) + C[3]*dT / (1<<7);
  P = ((D1 * sens) / (1<<21) - off) / (1<<15);

  result.temperature = temp / 100.0;
  result.P = P;
  result.delta = P - Pprev;
  result.dT = 0.02;
  Pprev = P;
}

/**
 *
 */
bool MS5806::start_t_measurement(void) {

  txbuf[0] = CMD_ADC_CONV | CMD_ADC_D2 | CMD_ADC_4096;

  if (MSG_OK != transmit(txbuf, 1, nullptr, 0))
    return OSAL_FAILED;
  else
    return OSAL_SUCCESS;
}

/**
 *
 */
bool MS5806::start_p_measurement(void) {

  txbuf[0] = CMD_ADC_CONV | CMD_ADC_D1 | CMD_ADC_4096;

  if (MSG_OK != transmit(txbuf, 1, nullptr, 0))
    return OSAL_FAILED;
  else
    return OSAL_SUCCESS;
}

/**
 *
 */
bool MS5806::acquire_data(uint8_t *rxbuf) {

  txbuf[0] = CMD_ADC_READ;

  if (MSG_OK != transmit(txbuf, 1, rxbuf, 3))
    return OSAL_FAILED;
  else
    return OSAL_SUCCESS;
}

/**
 *
 */
bool MS5806::hw_init_full(void) {
  msg_t status;

  for (size_t i=0; i<MS5806_CAL_WORDS; i++) {
    txbuf[0] = CMD_PROM_RD;
    txbuf[0] |= i << 1;
    status = transmit(txbuf, 1, (uint8_t*)&C[i], 2);
    osalDbgCheck(MSG_OK == status);
  }

  toggle_endiannes16((uint8_t *)C, sizeof(C));

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
__CCM__ static THD_WORKING_AREA(ms5806ThreadWA, 256);
THD_FUNCTION(ms5806Thread, arg) {
  chRegSetThreadName("ms5806");
  MS5806 *sensor = (MS5806 *)arg;
  systime_t starttime;
  baro_abs_data_t result;

  while (!chThdShouldTerminateX()) {
    if (SENSOR_STATE_READY == sensor->state) {
      starttime = chVTGetSystemTime();
      sensor->acquire_data(sensor->rxbuf_p);
      sensor->start_t_measurement();
      chThdSleepUntilWindowed(starttime, starttime + TEMP_CONV_TIME);

      starttime += TEMP_CONV_TIME;
      sensor->acquire_data(sensor->rxbuf_t);
      sensor->start_p_measurement();
      chThdSleepUntilWindowed(starttime, starttime + PRESS_CONV_TIME);

      sensor->calc_pressure(result);
      osalSysLock();
      sensor->cache = result;
      osalSysUnlock();
    }
    else {
      osalThreadSleep(TEMP_CONV_TIME + PRESS_CONV_TIME);
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
sensor_state_t MS5806::get(baro_abs_data_t &result) {
  osalDbgCheck(state == SENSOR_STATE_READY);

  result = cache;

  return this->state;
}
