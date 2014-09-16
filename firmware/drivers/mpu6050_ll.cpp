#include "main.h"
#include "mpu6050_ll.hpp"
#include "pack_unpack.hpp"
#include "utils.hpp"
#include "message.hpp"
#include "timekeeper.hpp"

#if MPU6050_1KHZ
#include "mpu6050_ll_fir_taps.h"
#endif

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
extern mavlink_raw_imu_t    mavlink_out_raw_imu_struct;
extern TimeKeeper time_keeper;

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
 *
 */
void MPU6050_LL::pickle_gyr(int16_t *result){
  uint8_t *b;

  // temperature
  b = &mpu_rxbuf[MPU_TEMP_OFFSET];
  result[3] = pack8to16be(b);

  // angular rate values
  b = &mpu_rxbuf[MPU_GYRO_OFFSET];
#if MPU6050_1KHZ
  int16_t tmp;
  for (size_t i=0; i<3; i++){
    tmp = pack8to16be(&b[i*2]);
    result[i] = gyr_fir[i].update(tmp);
  }
#else
  result[0] = pack8to16be(&b[0]);
  result[1] = pack8to16be(&b[2]);
  result[2] = pack8to16be(&b[4]);
#endif
  mavlink_out_raw_imu_struct.xgyro = result[0];
  mavlink_out_raw_imu_struct.ygyro = result[1];
  mavlink_out_raw_imu_struct.zgyro = result[2];
  mavlink_out_raw_imu_struct.zmag  = result[3]; // temperature hack
  mavlink_out_raw_imu_struct.time_usec = time_keeper.utc();
}

/**
 *
 */
void MPU6050_LL::pickle_acc(int16_t *result){
  uint8_t *b;

  // temperature
  b = &mpu_rxbuf[MPU_TEMP_OFFSET];
  result[3] = pack8to16be(b);

  //
  b = &mpu_rxbuf[MPU_ACCEL_OFFSET];
#if MPU6050_1KHZ
  int16_t tmp;
  for (size_t i=0; i<3; i++){
    tmp = pack8to16be(&b[i*2]);
    result[i] = acc_fir[i].update(tmp);
  }
#else
  result[0] = pack8to16be(&b[0]);
  result[1] = pack8to16be(&b[2]);
  result[2] = pack8to16be(&b[4]);
#endif
  mavlink_out_raw_imu_struct.xacc = result[0];
  mavlink_out_raw_imu_struct.yacc = result[1];
  mavlink_out_raw_imu_struct.zacc = result[2];
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
MPU6050_LL::MPU6050_LL(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr)
{
  #if MPU6050_1KHZ
  FIR<float, int16_t, ArrayLen(taps)> temp(taps, ArrayLen(taps));
  acc_fir[0] = temp;
  acc_fir[1] = temp;
  acc_fir[2] = temp;
  gyr_fir[0] = temp;
  gyr_fir[1] = temp;
  gyr_fir[2] = temp;
  #endif

  ready = false;
  hw_initialized = false;
}

/**
 *
 */
msg_t MPU6050_LL::hw_init_fast(void){
  ready = true;
  return RDY_OK; /* unimplemented */
}

/**
 *
 */
msg_t MPU6050_LL::hw_init_full(void){

  msg_t ret = RDY_RESET;

  mpu_txbuf[0] = MPU_PWR_MGMT1;
  mpu_txbuf[1] = 0b10000000; /* soft reset */
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  chDbgCheck(RDY_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(60);

  mpu_txbuf[0] = MPU_PWR_MGMT1;
  mpu_txbuf[1] = 1; /* select X gyro as clock source */
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  chDbgCheck(RDY_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(5);

  mpu_txbuf[0] = MPU_GYRO_CONFIG;
//  mpu_txbuf[1] = 0b00000; // 250 deg/s
  mpu_txbuf[1] = 0b01000; // 500 deg/s
//  mpu_txbuf[1] = 0b10000; // 1000 deg/s
//  mpu_txbuf[1] = 0b11000; // 2000 deg/s
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  chDbgCheck(RDY_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(1);

  mpu_txbuf[0] = MPU_ACCEL_CONFIG;
//  mpu_txbuf[1] = 0b00000; // 2g
//  mpu_txbuf[1] = 0b01000; // 4g
//  mpu_txbuf[1] = 0b10000; // 8g
  mpu_txbuf[1] = 0b11000; // 16g
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  chDbgCheck(RDY_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(1);

#if !MPU6050_1KHZ
  mpu_txbuf[0] = MPU_SMPLRT_DIV;
  /* sample rate. If (LPF > 0): (1000 / (val + 1))
   *                      else: (8000 / (val + 1)) */
  mpu_txbuf[1] = 9; /* val */

  /*    Bandwidth   Delay
  DLPF      (Hz)    (ms)
  1         188     1.9
  2         98      2.8
  3         42      4.8
  4         20      8.3
  5         10      13.4
  6         5       18.6
  7   reserved*/
  mpu_txbuf[2] = 1; /* LPF */
  transmit(mpu_txbuf, 3, NULL, 0);
  chThdSleepMilliseconds(5);
#else
  mpu_txbuf[0] = MPU_SMPLRT_DIV;
  mpu_txbuf[1] = 7; /* val */
  mpu_txbuf[2] = 0; /* LPF */
  ret = transmit(mpu_txbuf, 3, NULL, 0);
  chDbgCheck(RDY_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(5);
#endif

  mpu_txbuf[0] = MPU_INT_PIN_CFG;
  mpu_txbuf[1] = 0b00001000; /* clear int flag in register on any read operation */
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  chDbgCheck(RDY_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(1);

  mpu_txbuf[0] = MPU_INT_ENABLE;
  mpu_txbuf[1] = 1; /* enable data ready interrupts */
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  chDbgCheck(RDY_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(1);

  ready = true;

  return RDY_OK;
}

/**
 *
 */
msg_t MPU6050_LL::start(void){
  /* init hardware */
  if (! hw_initialized){
    if (need_full_init())
      hw_init_full();
    else
      hw_init_fast();
    hw_initialized = true;
  }

  return RDY_OK;
}

/**
 *
 */
void MPU6050_LL::stop(void){
  hw_initialized = false;
  ready = false;
}

/**
 *
 */
msg_t MPU6050_LL::update_gyr(int16_t *result){
  chDbgCheck((true == ready), "not ready");
  msg_t ret;

  mpu_txbuf[0] = MPU_INT_STATUS;
  ret = transmit(mpu_txbuf, 1, mpu_rxbuf, sizeof(mpu_rxbuf));
  this->pickle_gyr(result);

  return ret;
}

/**
 *
 */
msg_t MPU6050_LL::update_acc(int16_t *result){
  chDbgCheck((true == ready), "not ready");

  this->pickle_acc(result);
  return RDY_OK;
}



