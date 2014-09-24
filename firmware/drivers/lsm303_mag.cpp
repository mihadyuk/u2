#include "main.h"
#include "lsm303_mag.hpp"
#include "pack_unpack.h"
#include "marg2mavlink.hpp"
#include "param_registry.hpp"
#include "exti_local.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define LSM_REG_CRA           0x00
#define LSM_REG_CRB           0x01
#define LSM_REG_MR            0x02
#define LSM_REG_MAG_OUT       0x03
#define LSM_REG_ID            0x0A
#define LSM_REG_TEMP_OUT      0x31
#define GAIN_BITS_SHIFT       5

/**
 * @brief   Magnetometer gain (LSB/Gauss)
 */
typedef enum {
  LSM_MAG_GAIN_1370 = 0,
  LSM_MAG_GAIN_1090,
  LSM_MAG_GAIN_820,
  LSM_MAG_GAIN_660,
  LSM_MAG_GAIN_440,
  LSM_MAG_GAIN_390,
  LSM_MAG_GAIN_330,
  LSM_MAG_GAIN_230
} mag_sens_t;

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

static const float mag_sens_array[8] = {
    1.0f / 1370,
    1.0f / 1090,
    1.0f / 820,
    1.0f / 660,
    1.0f / 440,
    1.0f / 390,
    1.0f / 330,
    1.0f / 230
};

static bool mag_data_fresh = true;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 * @brief   convert parrots to Gauss.
 */
float LSM303_mag::mag_sens(void) {
  return mag_sens_array[*gain];
}

/**
 *
 */
void LSM303_mag::thermo_comp(float *result){
  (void)result;
}

/**
 *
 */
void LSM303_mag::iron_comp(float *result){
  (void)result;
}

/**
 *
 */
void LSM303_mag::pickle(float *result){

  int16_t raw[3];
  float sens = this->mag_sens();

  raw[0] = static_cast<int16_t>(pack8to16be(&rxbuf[0]));
  raw[1] = static_cast<int16_t>(pack8to16be(&rxbuf[2]));
  raw[2] = static_cast<int16_t>(pack8to16be(&rxbuf[4]));
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
msg_t LSM303_mag::hw_init_fast(void){
  return MSG_RESET; /* unimplemented */
}

/**
 *
 */
msg_t LSM303_mag::hw_init_full(void){

  msg_t ret = MSG_RESET;

  txbuf[0] = LSM_REG_ID;
  ret = transmit(txbuf, 1, rxbuf, 3);
  osalDbgAssert(MSG_OK == ret, "LSM303 magnetometer not responding");
  osalDbgAssert(0 == memcmp("H43", rxbuf, 3), "LSM303 magnetometer incorrect ID");

  txbuf[0] = LSM_REG_CRA;
  //txbuf[1] = 0b10011100; /* enable thermometer and set maximum output rate */
  txbuf[1] = 0;
  /* Set gain. 001 is documented for LSM303 and 000 is for HMC5883.
   * 000 looks working for LSM303 too - lets use it. */
  txbuf[2] = *gain << GAIN_BITS_SHIFT;
  /* single conversion mode */
  txbuf[3] = 0b00000001;

  ret = transmit(txbuf, 4, NULL, 0);
  osalDbgCheck(MSG_OK == ret);

  Exti.lsm303(true);

  return ret;
}

/**
 *
 */
msg_t LSM303_mag::start_single_measurement(void) {
  txbuf[0] = LSM_REG_MR;
  txbuf[1] = 0b00000001;
  return transmit(txbuf, 2, NULL, 0);
}

/**
 *
 */
msg_t LSM303_mag::set_gain(uint8_t val) {
  txbuf[0] = LSM_REG_CRB;
  txbuf[1] = val;
  return transmit(txbuf, 2, NULL, 0);
}

/**
 *
 */
msg_t LSM303_mag::refresh_gain(void) {

  uint8_t tmp = *gain;

  if (tmp != gain_prev) {
    gain_prev = tmp;
    return set_gain(tmp << GAIN_BITS_SHIFT);
  }
  else
    return MSG_OK;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
LSM303_mag::LSM303_mag(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr),
sample_cnt(0)
{
  ready = false;
}

/**
 *
 */
void LSM303_mag::stop(void){
  // TODO: power down sensor here
  ready = false;
}

/**
 *
 */
msg_t LSM303_mag::get(float *result){

  msg_t ret = MSG_OK;

  chDbgCheck(true == ready);

  if (nullptr != result) {
    if (true == mag_data_fresh) {
      txbuf[0] = LSM_REG_MAG_OUT;
      transmit(txbuf, 1, rxbuf, 6);
      pickle(result);
      memcpy(cache, result, sizeof(cache));

      mag_data_fresh = false;
      ret = start_single_measurement();
    }
    else{
      memcpy(result, cache, sizeof(cache));
    }
  }

  refresh_gain();

  return ret;
}

/**
 *
 */
msg_t LSM303_mag::start(void){
  msg_t ret;

  param_registry.valueSearch("LSMM_gain",  &gain);
  gain_prev = *gain;

  if (need_full_init())
    ret = hw_init_full();
  else
    ret = hw_init_fast();
  ready = true;

  return ret;
}

/**
 *
 */
void LSM303_mag::extiISR(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  mag_data_fresh = true;
}
