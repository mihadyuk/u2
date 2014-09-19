#include "main.h"
#include "mpu6050.hpp"
#include "pack_unpack.h"
#include "mavlink_local.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define MPU_ACCEL_OFFSET  1
#define MPU_TEMP_OFFSET   7
#define MPU_GYRO_OFFSET   9

#define MPUREG_SMPLRT_DIV       0x19
#define MPUREG_CONFIG           0x1A
#define MPUREG_GYRO_CONFIG      0x1B
#define MPUREG_ACCEL_CONFIG     0x1C
#define MPUREG_FIFO_EN          0x23
  #define FIFO_EN_BITS          0b1111000 /* accel and gyro */
#define MPUREG_INT_PIN_CFG      0x37
  #define I2C_BYPASS_EN         (1 << 1)
  #define INT_RD_CLEAR          (1 << 4) /* clear int flag in register on any read operation */
#define MPUREG_INT_ENABLE       0x38
#define MPUREG_INT_STATUS       0x3A /* 1 status bite and 14 bytes of data */
#define MPUREG_TEMP_OUT         0x41  /* MSB. Next byte is LSB */
#define MPUREG_USRE_CTRL        0x6A
  #define FIFO_EN               (1 << 6)
  #define I2C_MST_EN            (1 << 5) /* clear this bit to use I2C bypass mode */
  #define FIFO_RESET            (1 << 2)
#define MPUREG_PWR_MGMT1        0x6B
  #define DEVICE_RESET          (1 << 7)
#define MPUREG_PWR_MGMT2        0x6C
#define MPUREG_FIFO_CNT         0x72 /* MSB. Next byte is LSB */
#define MPUREG_FIFO_DATA        0x74
#define MPUREG_WHO_AM_I         0x75
  #define WHO_AM_I_VAL          0X68

/**
 * @brief   Gyro full scale in deg/s
 */
typedef enum {
  MPU_GYRO_FULL_SCALE_250   = 0,
  MPU_GYRO_FULL_SCALE_500   = 1,
  MPU_GYRO_FULL_SCALE_1000  = 2,
  MPU_GYRO_FULL_SCALE_2000  = 3
} gyro_sens_t;

/**
 * @brief   Accel full scale in g
 */
typedef enum {
  MPU_ACC_FULL_SCALE_2    = 0,
  MPU_ACC_FULL_SCALE_4    = 1,
  MPU_ACC_FULL_SCALE_8    = 2,
  MPU_ACC_FULL_SCALE_16   = 3,
} acc_sens_t;

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern mavlink_raw_imu_t        mavlink_out_raw_imu_struct;

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
#include "geometry.hpp"
static const float gyro_sens_array[4] = {
    deg2rad(250.0f  / 32768),
    deg2rad(500.0f  / 32768),
    deg2rad(1000.0f / 32768),
    deg2rad(2000.0f / 32768)
};

static const float acc_sens_array[4] = {
    (2 * 9.81)  / 32768.0,
    (4 * 9.81)  / 32768.0,
    (8 * 9.81)  / 32768.0,
    (16 * 9.81) / 32768.0
};

float MPU6050::gyr_sens(void){
  return gyro_sens_array[MPU_GYRO_FULL_SCALE_500];
}

/**
 *
 */
float MPU6050::acc_sens(void){
  return acc_sens_array[MPU_ACC_FULL_SCALE_16];
}

/**
 *
 */
static void acc2mavlink(int16_t *raw){
  mavlink_out_raw_imu_struct.xacc = raw[0];
  mavlink_out_raw_imu_struct.yacc = raw[1];
  mavlink_out_raw_imu_struct.zacc = raw[2];
  //mavlink_out_raw_imu_struct.time_usec = TimeKeeper::utc();
}

/**
 *
 */
static void gyr2mavlink(int16_t *raw){
  mavlink_out_raw_imu_struct.xgyro = raw[0];
  mavlink_out_raw_imu_struct.ygyro = raw[1];
  mavlink_out_raw_imu_struct.zgyro = raw[2];
  //mavlink_out_raw_imu_struct.time_usec = TimeKeeper::utc();
}

/**
 *
 */
void MPU6050::pickle_temp(float *result){
  uint8_t *b = &mpu_rxbuf[MPU_TEMP_OFFSET];
  result[0] = static_cast<int16_t>(pack8to16be(b));
  result[0] /= 340;
  result[0] += 36.53f;
}

/**
 *
 */
void MPU6050::gyro_thermo_comp(float *result){
  (void)result;
}

/**
 *
 */
void MPU6050::acc_egg_comp(float *result){
  (void)result;
}

/**
 *
 */
void MPU6050::pickle_gyr(float *result){

  int16_t raw[3];
  uint8_t *b = &mpu_rxbuf[MPU_GYRO_OFFSET];
  float sens = this->gyr_sens();

  raw[0] = static_cast<int16_t>(pack8to16be(&b[0]));
  raw[1] = static_cast<int16_t>(pack8to16be(&b[2]));
  raw[2] = static_cast<int16_t>(pack8to16be(&b[4]));
  gyr2mavlink(raw);

  result[0] = sens * raw[0];
  result[1] = sens * raw[1];
  result[2] = sens * raw[2];

  gyro_thermo_comp(result);
}

/**
 *
 */
void MPU6050::pickle_acc(float *result){

  int16_t raw[3];
  uint8_t *b = &mpu_rxbuf[MPU_ACCEL_OFFSET];
  float sens = this->acc_sens();

  raw[0] = static_cast<int16_t>(pack8to16be(&b[0]));
  raw[1] = static_cast<int16_t>(pack8to16be(&b[2]));
  raw[2] = static_cast<int16_t>(pack8to16be(&b[4]));
  acc2mavlink(raw);

  result[0] = sens * raw[0];
  result[1] = sens * raw[1];
  result[2] = sens * raw[2];

  acc_egg_comp(result);
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

  mpu_txbuf[0] = MPUREG_PWR_MGMT1;
  mpu_txbuf[1] = 0b10000000; /* soft reset */
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  osalThreadSleepMilliseconds(60);

  mpu_txbuf[0] = MPUREG_WHO_AM_I;
  ret = transmit(mpu_txbuf, 1, mpu_rxbuf, 1);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  osalDbgAssert(WHO_AM_I_VAL == mpu_rxbuf[0], "MPU6050 wrong id");
  osalThreadSleepMilliseconds(1);

  mpu_txbuf[0] = MPUREG_PWR_MGMT1;
  mpu_txbuf[1] = 1; /* select X gyro as clock source */
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  osalThreadSleepMilliseconds(5);

  mpu_txbuf[0] = MPUREG_GYRO_CONFIG;
  mpu_txbuf[1] = MPU_GYRO_FULL_SCALE_500 << 3;
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  osalThreadSleepMilliseconds(1);

  mpu_txbuf[0] = MPUREG_ACCEL_CONFIG;
  mpu_txbuf[1] = MPU_ACC_FULL_SCALE_16 << 3;
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  osalThreadSleepMilliseconds(1);

#if !MPU6050_1KHZ
  mpu_txbuf[0] = MPUREG_SMPLRT_DIV;
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
  osalThreadSleepMilliseconds(5);
#else
  mpu_txbuf[0] = MPUREG_SMPLRT_DIV;
  mpu_txbuf[1] = 7; /* val */
  mpu_txbuf[2] = 0; /* LPF */
  ret = transmit(mpu_txbuf, 3, NULL, 0);
  chDbgCheck(RDY_OK == ret, "MPU6050 does not responding");
  osalThreadSleepMilliseconds(5);
#endif

  mpu_txbuf[0] = MPUREG_INT_PIN_CFG;
  mpu_txbuf[1] = INT_RD_CLEAR | I2C_BYPASS_EN;
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  osalThreadSleepMilliseconds(1);

  mpu_txbuf[0] = MPUREG_INT_ENABLE;
  mpu_txbuf[1] = 1; /* enable data ready interrupts */
  ret = transmit(mpu_txbuf, 2, NULL, 0);
  osalDbgAssert(MSG_OK == ret, "MPU6050 does not responding");
  osalThreadSleepMilliseconds(1);

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

  mpu_txbuf[0] = MPUREG_INT_STATUS;
  ret = transmit(mpu_txbuf, 1, mpu_rxbuf, sizeof(mpu_rxbuf));

  pickle_temp(&temp);

  if (nullptr != gyr)
    pickle_gyr(gyr);

  if (nullptr != acc)
    pickle_acc(acc);

  return ret;
}

/**
 *
 */
float MPU6050::update_perod(void){
  return 0.01;
}

