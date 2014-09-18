#include "main.h"
#include "mpu6050.hpp"
#include "pack_unpack.h"
#include "param_registry.hpp"
#include "mavlink_local.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define MPU_ACCEL_OFFSET  1
#define MPU_TEMP_OFFSET   7
#define MPU_GYRO_OFFSET   9

#define MPU_SMPLRT_DIV    0x19
#define MPU_CONFIG        0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_INT_PIN_CFG   0x37
#define MPU_INT_ENABLE    0x38
#define MPU_INT_STATUS    0x3A /* 1 status bite and 14 bytes of data */
#define MPU_PWR_MGMT1     0x6B
#define MPU_PWR_MGMT2     0x6C

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_raw_imu_t    mavlink_out_raw_imu_struct;
//extern TimeKeeper time_keeper;

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
void MPU6050::pickle_gyr(float *result){
  uint8_t *b;

  // temperature
//  b = &mpu_rxbuf[MPU_TEMP_OFFSET];
//  result[3] = pack8to16be(b);

  // angular rate values
  b = &mpu_rxbuf[MPU_GYRO_OFFSET];
  result[0] = pack8to16be(&b[0]);
  result[1] = pack8to16be(&b[2]);
  result[2] = pack8to16be(&b[4]);

  mavlink_out_raw_imu_struct.xgyro = result[0];
  mavlink_out_raw_imu_struct.ygyro = result[1];
  mavlink_out_raw_imu_struct.zgyro = result[2];
}

/**
 *
 */
void MPU6050::pickle_acc(float *result){
  uint8_t *b;

  // temperature
//  b = &mpu_rxbuf[MPU_TEMP_OFFSET];
//  result[3] = pack8to16be(b);

  //
  b = &mpu_rxbuf[MPU_ACCEL_OFFSET];
  result[0] = pack8to16be(&b[0]);
  result[1] = pack8to16be(&b[2]);
  result[2] = pack8to16be(&b[4]);

  mavlink_out_raw_imu_struct.xacc = result[0];
  mavlink_out_raw_imu_struct.yacc = result[1];
  mavlink_out_raw_imu_struct.zacc = result[2];
}

/**
 *
 */
msg_t MPU6050::hw_init_fast(void){
  ready = true;
  return MSG_OK; /* unimplemented */
}

/**
 *
 */
msg_t MPU6050::hw_init_full(void){

  msg_t ret = MSG_RESET;

  mpu_txbuf[0] = MPU_PWR_MGMT1;
  mpu_txbuf[1] = 0b10000000; /* soft reset */
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(60);

  mpu_txbuf[0] = MPU_PWR_MGMT1;
  mpu_txbuf[1] = 1; /* select X gyro as clock source */
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(5);

  mpu_txbuf[0] = MPU_GYRO_CONFIG;
//  mpu_txbuf[1] = 0b00000; // 250 deg/s
  mpu_txbuf[1] = 0b01000; // 500 deg/s
//  mpu_txbuf[1] = 0b10000; // 1000 deg/s
//  mpu_txbuf[1] = 0b11000; // 2000 deg/s
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(1);

  mpu_txbuf[0] = MPU_ACCEL_CONFIG;
//  mpu_txbuf[1] = 0b00000; // 2g
//  mpu_txbuf[1] = 0b01000; // 4g
//  mpu_txbuf[1] = 0b10000; // 8g
  mpu_txbuf[1] = 0b11000; // 16g
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
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
  mpu_txbuf[2] = 5; /* LPF */
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
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(1);

  mpu_txbuf[0] = MPU_INT_ENABLE;
  mpu_txbuf[1] = 1; /* enable data ready interrupts */
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  chThdSleepMilliseconds(1);

  ready = true;

  return MSG_OK;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
MPU6050::MPU6050(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr)
{
  ready = false;
  hw_initialized = false;
}

/**
 *
 */
msg_t MPU6050::start(void){
  /* init hardware */
  if (! hw_initialized){
    if (need_full_init())
      hw_init_full();
    else
      hw_init_fast();
    hw_initialized = true;
  }

  return MSG_OK;
}

/**
 *
 */
void MPU6050::stop(void){
  hw_initialized = false;
  ready = false;
}

/**
 *
 */
msg_t MPU6050::get(float *acc, float *gyr) {
  osalDbgCheck(true == ready);
  msg_t ret = MSG_RESET;

  mpu_txbuf[0] = MPU_INT_STATUS;
  ret = transmit(mpu_txbuf, 1, mpu_rxbuf, sizeof(mpu_rxbuf));

  if (nullptr != gyr)
    pickle_gyr(gyr);

  if (nullptr != acc)
    pickle_acc(acc);

  return ret;
}



