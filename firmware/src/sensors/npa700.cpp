#include <cmath>

#include "main.h"
#include "pads.h"
#include "param_registry.hpp"

#include "baro_data.hpp"
#include "npa700.hpp"

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

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/* constants got from Application Guide */
//static const int32_t mid_point = 8192;
//static const int32_t mid_point = 8171;
//static const float sens = 5.3402502288; // (35000 / 6554);

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
/**
 *
 */
void NPA700::pickle(baro_diff_data_t &result) {
  uint16_t raw;
  int32_t temp;

  temp = rxbuf[2];
  temp = -50 + (temp * 200) / 256;

  raw  = (rxbuf[0] << 8) | rxbuf[1];

  // diagnostic
  uint8_t diag = rxbuf[0] >> 6;
  if ((3 == diag) && (1 == diag)) {
    this->state = SENSOR_STATE_DEAD;
    return;
  }
  else {
    float pres;
    pres  = raw & 0x3FFF; // cut status bits out
    pres -= *mid;
    pres *= *sens;

    result.p   = roundf(pres);
    result.raw = raw;
    result.t   = temp;
  }
}

/**
 *
 */
NPA700::NPA700(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr)
{
  state = SENSOR_STATE_STOP;
}

/**
 * @brief   Just check device's presence on bus
 */
bool NPA700::hw_init_full(void) {
  msg_t status = MSG_RESET;

  status = receive(rxbuf, 3);

  if (MSG_OK == status)
    return OSAL_SUCCESS;
  else
    return OSAL_FAILED;
}

/**
 * @brief   Just check device's presence on bus
 */
bool NPA700::ping(void) {
  msg_t status = receive(rxbuf, 3);

  if (MSG_OK == status)
    return OSAL_SUCCESS;
  else
    return OSAL_FAILED;
}

/**
 * @brief   Dummy function
 */
bool NPA700::hw_init_fast(void) {
  return OSAL_SUCCESS;
}

/**
 *
 */
sensor_state_t NPA700::start(void){

  param_registry.valueSearch("PMU_npa700_mid",  &mid);
  param_registry.valueSearch("PMU_npa700_sens", &sens);

  state = SENSOR_STATE_READY;
  return state;
}

/**
 *
 */
void NPA700::stop(void){
  state = SENSOR_STATE_STOP;
}

/**
 *
 */
sensor_state_t NPA700::wakeup(void) {
  state = SENSOR_STATE_READY;
  return state;
}

/**
 *
 */
void NPA700::sleep(void) {
  state = SENSOR_STATE_SLEEP;
}

/**
 *
 */
sensor_state_t NPA700::get(baro_diff_data_t &result){

  if (SENSOR_STATE_READY == state) {
    msg_t status = MSG_RESET;
    status = receive(rxbuf, 4);

    if (MSG_OK == status)
      this->pickle(result);
    else
      state = SENSOR_STATE_DEAD;
  }

  return state;
}
