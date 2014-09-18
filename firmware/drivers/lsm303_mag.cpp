#include <math.h>

#include "main.h"
#include "lsm303_mag_ll.hpp"
#include "pack_unpack.hpp"

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

/**
 *
 */
void LSM303_mag_LL::pickle(int16_t *result){

  // magnetinc flux
  result[0] = pack8to16be(&rxbuf[0]);
  result[1] = pack8to16be(&rxbuf[2]);
  result[2] = pack8to16be(&rxbuf[4]);

  // temperature
  result[3] = pack8to16be(&rxbuf[6]);
}

/**
 *
 */
msg_t LSM303_mag_LL::hw_init_fast(void){
  return RDY_RESET; /* unimplemented */
}

/**
 *
 */
msg_t LSM303_mag_LL::hw_init_full(void){

  txbuf[0] = CRA_REG_M;
  /* enable thermometer and set maximum output rate */
  txbuf[1] = 0b10011100;
  /* set maximum gain. 001 is documented for LSM303 and 000 is for HMC5883.
   * 000 looks working for LSM303 too. */
  txbuf[2] = 0b00100000;
  //txbuf[2] = 0b00000000;
  /* single conversion mode */
  txbuf[3] = 0b00000001;

  return transmit(txbuf, 4, NULL, 0);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
LSM303_mag_LL::LSM303_mag_LL(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr),
sample_cnt(0)
{
  ready = false;
}

/**
 *
 */
void LSM303_mag_LL::stop(void){
  // TODO: power down sensor here
  ready = false;
}

/**
 *
 */
msg_t LSM303_mag_LL::update(int16_t *result){
  msg_t ret;

  chDbgCheck((true == ready), "not ready");

  if ((sample_cnt % 128) == 0){
    txbuf[0] = TEMP_OUT_H_M;
    ret = transmit(txbuf, 1, rxbuf+6, 2);
    if (RDY_OK != ret)
      return ret;
  }
  sample_cnt++;

  /* read previose measurement results */
  txbuf[0] = OUT_X_H_M;
  ret = transmit(txbuf, 1, rxbuf, 6);
  if (RDY_OK != ret)
    return ret;

  /* start new measurement */
  txbuf[0] = MR_REG_M;
  txbuf[1] = 0b00000001;
  ret = transmit(txbuf, 2, NULL, 0);
  if (RDY_OK != ret)
    return ret;

  this->pickle(result);

  return ret;
}

/**
 *
 */
msg_t LSM303_mag_LL::start(void){
  msg_t ret;

  if (need_full_init())
    ret = hw_init_full();
  else
    ret = hw_init_fast();
  ready = true;

  return ret;
}
