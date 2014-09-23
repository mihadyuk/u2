#include "main.h"

#include "lsm303_acc.hpp"
#include "pack_unpack.h"
#include "marg2mavlink.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
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
  return acc_sens_array[LSM_ACC_FULL_SCALE_8];
}

/**
 *
 */
void LSM303_acc::pickle(float *result){

  int16_t raw[3];
  float sens = this->acc_sens();

  raw[0] = static_cast<int16_t>(pack8to16be(&rxbuf[0]));
  raw[1] = static_cast<int16_t>(pack8to16be(&rxbuf[2]));
  raw[2] = static_cast<int16_t>(pack8to16be(&rxbuf[4]));
  acc2raw_imu(raw);

  /* */
  result[0] = sens * raw[0];
  result[1] = sens * raw[1];
  result[2] = sens * raw[2];
}

/**
 *
 */
msg_t LSM303_acc::hw_init_fast(void){
  return MSG_RESET;
}

/**
 *
 */
msg_t LSM303_acc::hw_init_full(void){

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
  tmp |= LSM_ACC_FULL_SCALE_8 << 4;
  txbuf[4] = tmp;

  /* REG5: */
  txbuf[5] = 0b0;

  /* REG6: */
  txbuf[6] = 0b0;

  ret = transmit(txbuf, 7, NULL, 0);
  osalDbgCheck(MSG_OK == ret);
  return ret;
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
  ready = false;
}

/**
 *
 */
void LSM303_acc::stop(void){
  // TODO: power down sensor here
  ready = false;
}

/**
 *
 */
msg_t LSM303_acc::get(float *result){

  chDbgCheck(true == ready);

  msg_t ret;

  /* read previose measurement results */
  txbuf[0] = OUT_X_L_A | 0b10000000;
  ret = transmit(txbuf, 1, rxbuf, 6);
  this->pickle(result);

  return ret;
}

/**
 *
 */
msg_t LSM303_acc::start(void){

  msg_t ret;

  if (need_full_init())
    ret = hw_init_full();
  else
    ret = hw_init_fast();
  ready = true;

  return ret;
}
