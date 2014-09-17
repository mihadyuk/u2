#include <math.h>

#include "main.h"
#include "lsm303_acc.hpp"

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
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

///**
// *
// */
//void LSM303_acc_LL::pickle(int16_t *result){
//
//  result[0] = pack8to16be(&rxbuf[0]);
//  result[1] = pack8to16be(&rxbuf[2]);
//  result[2] = pack8to16be(&rxbuf[4]);
//}

/**
 *
 */
msg_t LSM303_acc_LL::hw_init_fast(void){
  return MSG_RESET;
}

/**
 *
 */
msg_t LSM303_acc_LL::hw_init_full(void){

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
   * 8g full scale
   * high resolution */
  txbuf[4] = 0b01101000;

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
LSM303_acc_LL::LSM303_acc_LL(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr)
{
  ready = false;
}

/**
 *
 */
void LSM303_acc_LL::stop(void){
  // TODO: power down sensor here
  ready = false;
}

/**
 *
 */
msg_t LSM303_acc_LL::update(int16_t *result){

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
msg_t LSM303_acc_LL::start(void){

  msg_t ret;

  if (need_full_init())
    ret = hw_init_full();
  else
    ret = hw_init_fast();
  ready = true;

  return ret;
}
