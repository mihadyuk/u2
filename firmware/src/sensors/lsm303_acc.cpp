#include "main.h"

#include "lsm303_acc.hpp"
#include "pack_unpack.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define CTRL_REG1_A             0x20
#define CTRL_REG2_A             0x21
#define OUT_X_L_A               0x28

/**
 * @brief   Accel full scale in g
 */
typedef enum {
  LSM_ACC_FULL_SCALE_2 = 0,
  LSM_ACC_FULL_SCALE_4,
  LSM_ACC_FULL_SCALE_8,
  LSM_ACC_FULL_SCALE_16
} acc_sens_t;

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

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

static const acc_sens_t DEFAULT_FULL_SCALE = LSM_ACC_FULL_SCALE_8;

static const float acc_sens_array[4] = {
    (2 * 9.81)  / 32768.0,
    (4 * 9.81)  / 32768.0,
    (8 * 9.81)  / 32768.0,
    (16 * 9.81) / 32768.0
};

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 *
 */
float LSM303_acc::acc_sens(void){
  return acc_sens_array[DEFAULT_FULL_SCALE];
}

/**
 *
 */
void LSM303_acc::pickle(float *result, int16_t *result_raw){

  int16_t raw[3];
  float sens = this->acc_sens();

  for (size_t i=0; i<3; i++) {
    raw[i] = static_cast<int16_t>(pack8to16be(&rxbuf[i*2]));
    result[i] = sens * raw[i];
    result_raw[i] = raw[i];
  }
}

/**
 *
 */
bool LSM303_acc::hw_init_fast(void){
  return hw_init_full();
}

/**
 *
 */
bool LSM303_acc::hw_init_full(void){

  msg_t ret = MSG_RESET;

  txbuf[0] = CTRL_REG1_A | 0b10000000;

  /* REG1:
   * enable axis and set output rate */
  txbuf[1] = 0b10010111;

  /* REG2: */
  txbuf[2] = 0b0;

  /* REG3: */
  txbuf[3] = 0b0;

  /* REG4:
   * continuous update
   * net byte order
   * high resolution */
  uint8_t tmp;
  tmp = 0b01001000;
  tmp |= DEFAULT_FULL_SCALE << 4;
  txbuf[4] = tmp;

  /* REG5: */
  txbuf[5] = 0b0;

  /* REG6: */
  txbuf[6] = 0b0;

  ret = transmit(txbuf, 7, NULL, 0);
  if (MSG_OK == ret)
    return OSAL_SUCCESS;
  else
    return OSAL_FAILED;
}

/**
 *
 */
msg_t LSM303_acc::stop_sleep_code(void) {
  txbuf[0] = CTRL_REG1_A | 0b10000000;
  txbuf[1] = 0;
  return transmit(txbuf, 2, NULL, 0);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
LSM303_acc::LSM303_acc(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr)
{
  state = SENSOR_STATE_STOP;
}

/**
 *
 */
sensor_state_t LSM303_acc::start(void){

  if (SENSOR_STATE_STOP == this->state) {

    /* init hardware */
    bool init_status = OSAL_FAILED;
    if (need_full_init())
      init_status = hw_init_full();
    else
      init_status = hw_init_fast();

    /* check state */
    if (OSAL_SUCCESS == init_status)
      this->state = SENSOR_STATE_READY;
    else
      this->state = SENSOR_STATE_DEAD;
  }

  return this->state;
}

/**
 *
 */
void LSM303_acc::stop(void) {

  if ((this->state == SENSOR_STATE_STOP) || (this->state == SENSOR_STATE_DEAD))
    return;
  else {
    if (MSG_OK == stop_sleep_code())
      this->state = SENSOR_STATE_STOP;
    else
      this->state = SENSOR_STATE_DEAD;
  }
}

/**
 *
 */
sensor_state_t LSM303_acc::get(marg_data_t &result) {

  if ((SENSOR_STATE_READY == this->state) && (1 == result.request.acc)) {

    /* read previose measurement results */
    txbuf[0] = OUT_X_L_A | 0b10000000;
    if (MSG_OK != transmit(txbuf, 1, rxbuf, 6))
      this->state = SENSOR_STATE_DEAD;
    else
      this->pickle(result.acc, result.acc_raw);
  }

  return this->state;
}

/**
 *
 */
void LSM303_acc::sleep(void) {
  if (this->state == SENSOR_STATE_SLEEP)
    return;

  osalDbgAssert(this->state == SENSOR_STATE_READY, "Invalid state");
  if (MSG_OK != stop_sleep_code())
    this->state = SENSOR_STATE_DEAD;
  else
    this->state = SENSOR_STATE_SLEEP;
}

/**
 *
 */
sensor_state_t LSM303_acc::wakeup(void) {
  if (this->state == SENSOR_STATE_READY)
    return this->state;

  osalDbgAssert(this->state == SENSOR_STATE_SLEEP, "Invalid state");

  /* FIXME: this code may be faster */
  if (OSAL_SUCCESS == hw_init_fast())
    this->state = SENSOR_STATE_READY;
  else
    this->state = SENSOR_STATE_DEAD;
  return this->state;
}



