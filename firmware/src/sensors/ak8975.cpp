#include "main.h"
#include "ak8975.hpp"
#include "pack_unpack.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define MEASUREMENT_TIME    MS2ST(9)

#define AKREG_WIA         0x00 /* who am I register, always contains 0x48 */
  #define WIA_VAL         0x48
#define AKREG_ST1         0x02
  #define ST1_DATA_READY  0x01
#define AKREG_HXL         0x03  /* first register containing measurement */
#define AKREG_CNTL        0x0A
  #define CNTL_SS         0x01  /* single shot mode */

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
 * @brief   convert parrots to Gauss.
 */
float AK8975::mag_sens(void) {
  return 0.003;
}

/**
 *
 */
void AK8975::thermo_comp(marg_vector_t &result){
  (void)result;
}

/**
 *
 */
void AK8975::iron_comp(marg_vector_t &result){
  (void)result;
}

/**
 *
 */
void AK8975::pickle(marg_vector_t &result, marg_vector_raw_t &result_raw) {

  int16_t raw[3];
  float sens = this->mag_sens();

  for (size_t i=0; i<3; i++) {
    raw[i] = static_cast<int16_t>(pack8to16le(&rxbuf[i*2 + 1]));
    result[i] = sens * raw[i];
    result_raw[i] = raw[i];
  }

  thermo_comp(result);
  iron_comp(result);
}

/**
 *
 */
bool AK8975::hw_init_fast(void) {
  return hw_init_full();
}

/**
 *
 */
bool AK8975::hw_init_full(void) {

  msg_t ret = MSG_RESET;
  uint8_t txbuf[1];

  txbuf[0] = AKREG_WIA;
  ret = transmit(txbuf, 1, rxbuf, 2);
  if (MSG_OK != ret) // AK8975 does not responding. Ensure that you start MPU6050 first
    return OSAL_FAILED;
  if (WIA_VAL != rxbuf[0]) // AK8975 incorrect ACK
    return OSAL_FAILED;

  return OSAL_SUCCESS;
}

/**
 *
 */
msg_t AK8975::start_measurement(void) {

  msg_t ret = MSG_RESET;
  uint8_t txbuf[2];

  txbuf[0] = AKREG_CNTL;
  txbuf[1] = CNTL_SS;
  ret = transmit(txbuf, 2, NULL, 0);

  return ret;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
AK8975::AK8975(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr)
{
  state = SENSOR_STATE_STOP;
}

/**
 *
 */
sensor_state_t AK8975::start(void) {

  if (SENSOR_STATE_STOP == this->state) {
    bool init_status;
    if (need_full_init())
      init_status = hw_init_full();
    else
      init_status = hw_init_fast();

    /* check state */
    if (OSAL_SUCCESS != init_status) {
      this->state = SENSOR_STATE_DEAD;
      return this->state;
    }

    /* kick start measurement */
    if (MSG_OK != start_measurement()) {
      this->state = SENSOR_STATE_DEAD;
      return this->state;
    }

    this->state = SENSOR_STATE_READY;
  }

  return this->state;
}

/**
 *
 */
void AK8975::stop(void) {
  if (this->state == SENSOR_STATE_STOP)
    return;

  osalDbgAssert((this->state == SENSOR_STATE_READY) ||
                (this->state == SENSOR_STATE_SLEEP), "Invalid state");
  this->state = SENSOR_STATE_STOP;
}

/**
 *
 */
void AK8975::sleep(void) {
  if (this->state == SENSOR_STATE_SLEEP)
    return;

  osalDbgAssert(this->state == SENSOR_STATE_READY, "Invalid state");
  this->state = SENSOR_STATE_SLEEP;
}

/**
 *
 */
sensor_state_t AK8975::wakeup(void) {
  if (this->state == SENSOR_STATE_READY)
    return this->state;

  osalDbgAssert(this->state == SENSOR_STATE_SLEEP, "Invalid state");
  this->state = SENSOR_STATE_READY;
  return this->state;
}

/**
 * @brief   Return magnentometer data in Gauss
 */
sensor_state_t AK8975::get(marg_data_t &result) {

  uint8_t txbuf[2];

  if ((this->state == SENSOR_STATE_READY) && (1 == result.request.mag)) {
    txbuf[0] = AKREG_ST1;
    if (MSG_OK != transmit(txbuf, 1, rxbuf, 7)) {
      this->state = SENSOR_STATE_DEAD;
      return this->state;
    }

    /* protection from too fast data acquisition */
    if (ST1_DATA_READY == (rxbuf[0] & ST1_DATA_READY)) {
      /* read measured data and immediately start new measurement */
      pickle(result.mag, result.mag_raw);
      cache = result.mag;
      cache_raw = result.mag_raw;
      if (MSG_OK != start_measurement()) {
        this->state = SENSOR_STATE_DEAD;
        return this->state;
      }
    }
    else {
      result.mag = cache;
      result.mag_raw = cache_raw;
    }
  }

  return this->state;
}

