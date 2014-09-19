#include "main.h"
#include "lsm303_mag.hpp"
#include "pack_unpack.h"
#include "mavlink_local.hpp"

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

extern mavlink_raw_imu_t        mavlink_out_raw_imu_struct;

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
static void mag2mavlink(int16_t *raw){
  mavlink_out_raw_imu_struct.xmag = raw[0];
  mavlink_out_raw_imu_struct.ymag = raw[1];
  mavlink_out_raw_imu_struct.zmag = raw[2];
  //mavlink_out_raw_imu_struct.time_usec = TimeKeeper::utc();
}

/**
 * @brief   convert parrots to Gauss.
 */
float LSM303_mag::mag_sens(void) {
  return mag_sens_array[LSM_MAG_GAIN_1370];
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

  raw[0] = static_cast<int16_t>(pack8to16be(&rxbuf[1]));
  raw[1] = static_cast<int16_t>(pack8to16be(&rxbuf[3]));
  raw[2] = static_cast<int16_t>(pack8to16be(&rxbuf[5]));
  mag2mavlink(raw);

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
  ret = transmit(txbuf, 4, rxbuf, 3);
  osalDbgAssert(MSG_OK == ret, "LSM303 magnetometer not responding");
  osalDbgAssert(0 == memcmp("H43", rxbuf, 3), "LSM303 magnetometer incorrect ID");

  txbuf[0] = LSM_REG_CRA;
  /* enable thermometer and set maximum output rate */
  txbuf[1] = 0b10011100;
  /* Set gain. 001 is documented for LSM303 and 000 is for HMC5883.
   * 000 looks working for LSM303 too - lets use it. */
  txbuf[2] = LSM_MAG_GAIN_1370 << 5;
  /* single conversion mode */
  txbuf[3] = 0b00000001;

  ret = transmit(txbuf, 4, NULL, 0);
  osalDbgCheck(MSG_OK == ret);

  ret = start_single_measurement();

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
  msg_t ret;

  chDbgCheck(true == ready);

  if ((sample_cnt % 128) == 0){
    txbuf[0] = LSM_REG_TEMP_OUT;
    ret = transmit(txbuf, 1, rxbuf+6, 2);
    if (MSG_OK != ret)
      return ret;
  }
  sample_cnt++;

  /* read previous measurement results */
  txbuf[0] = LSM_REG_MAG_OUT;
  ret = transmit(txbuf, 1, rxbuf, 7);
  if (MSG_OK != ret)
    return ret;

  if (nullptr != result){
    if (1 == (rxbuf[6] & 1)) { /* if data ready bit set */
      pickle(result);
      ret = start_single_measurement();
    }
    else{
      memcpy(result, cache, sizeof(cache));
    }
  }

  return ret;
}

/**
 *
 */
msg_t LSM303_mag::start(void){
  msg_t ret;

  if (need_full_init())
    ret = hw_init_full();
  else
    ret = hw_init_fast();
  ready = true;

  return ret;
}
