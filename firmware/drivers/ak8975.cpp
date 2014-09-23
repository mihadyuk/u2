#include "main.h"
#include "ak8975.hpp"
#include "pack_unpack.h"
#include "marg2mavlink.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define MEASUREMENT_TIME    MS2ST(9)

#define AKREG_WIA         0x00 /* who am I register, always contain 0x48 */
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
void AK8975::thermo_comp(float *result){
  (void)result;
}

/**
 *
 */
void AK8975::iron_comp(float *result){
  (void)result;
}

/**
 *
 */
void AK8975::pickle(float *result){

  int16_t raw[3];
  float sens = this->mag_sens();

  raw[0] = static_cast<int16_t>(pack8to16le(&rxbuf[1]));
  raw[1] = static_cast<int16_t>(pack8to16le(&rxbuf[3]));
  raw[2] = static_cast<int16_t>(pack8to16le(&rxbuf[5]));
  mag2raw_imu(raw);

  /* */
  result[0] = sens * raw[0];
  result[1] = sens * raw[1];
  result[2] = sens * raw[2];
  thermo_comp(result);
  iron_comp(result);
}

/**
 *
 */
msg_t AK8975::hw_init_fast(void){
  ready = true;
  return MSG_OK; /* unimplemented */
}

/**
 *
 */
msg_t AK8975::hw_init_full(void){

  msg_t ret = MSG_RESET;
  uint8_t txbuf[1];

  txbuf[0] = AKREG_WIA;
  ret = transmit(txbuf, 1, rxbuf, 2);
  osalDbgAssert(MSG_OK == ret, "AK8975 does not responding. Ensure you start MPU6050 first");
  osalDbgAssert(WIA_VAL == rxbuf[0], "AK8975 incorrect ACK");

  ready = true;
  return MSG_OK;
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
  ready = false;
}

/**
 *
 */
msg_t AK8975::start(void){

  if (need_full_init())
    hw_init_full();
  else
    hw_init_fast();

  start_measurement();
  return MSG_OK;
}

/**
 *
 */
void AK8975::stop(void){
  ready = false;
}

/**
 * @brief   Return magnentometer data in Gauss
 */
msg_t AK8975::get(float *mag) {

  msg_t ret = MSG_RESET;
  uint8_t txbuf[2];

  osalDbgCheck(true == ready);

  txbuf[0] = AKREG_ST1;
  ret = transmit(txbuf, 1, rxbuf, 7);

  if (nullptr != mag){
    /* protection from too fast data acquisition */
    if (ST1_DATA_READY == (rxbuf[0] & ST1_DATA_READY)){
      /* read measured data and immediately start new measurement */
      pickle(mag);
      start_measurement();
    }
    else
      memcpy(mag, cache, sizeof(cache));
  }

  return ret;
}




