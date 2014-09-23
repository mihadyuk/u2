#include "main.h"
#include "exti_local.hpp"
#include "mpu6050.hpp"
#include "ak8975.hpp"
#include "lsm303_acc.hpp"
#include "lsm303_mag.hpp"
#include "mavlink_local.hpp"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/**
 * @brief   Acceleration measurement source
 */
typedef enum {
  MARG_ACC_SRC_MPU6050 = 0,
  MARG_ACC_SRC_ADIS,
  MARG_ACC_SRC_LSM303,
  MARG_ACC_SRC_NONE,
} acc_src_t;

/**
 * @brief   Angular rate measurement source
 */
typedef enum {
  MARG_GYR_SRC_MPU6050 = 0,
  MARG_GYR_SRC_ADIS,
  MARG_GYR_SRC_NONE,
} gyr_src_t;

/**
 * @brief   Magnetic measurement source
 */
typedef enum {
  MARG_MAG_SRC_LSM303 = 0,
  MARG_MAG_SRC_AK8975,
  MARG_MAG_SRC_ADIS,
  MARG_MAG_SRC_NONE,
} mag_src_t;

/**
 * @brief   Synchronization source will be deduced based on gyro source.
 */
typedef enum {
} sync_src_t;

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern mavlink_highres_imu_t    mavlink_out_highres_imu_struct;

//extern TimeKeeper time_keeper;

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

static BinarySemaphore marg_sem(true);

static MPU6050 mpu6050(&I2CD_FAST, mpu6050addr);
static AK8975 ak8975(&I2CD_FAST, ak8975addr);
static LSM303_mag lsm303mag(&I2CD_FAST, lsm303magaddr);
static LSM303_acc lsm303acc(&I2CD_FAST, lsm303accaddr);

static float acc_data[3];
static float gyr_data[3];
static float mag_data[3];

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
static void marg2mavlink(float *acc, float *gyr, float *mag){

  if (nullptr != acc){
    mavlink_out_highres_imu_struct.xacc = acc[0];
    mavlink_out_highres_imu_struct.yacc = acc[1];
    mavlink_out_highres_imu_struct.zacc = acc[2];
  }

  if (nullptr != gyr){
    mavlink_out_highres_imu_struct.xgyro = gyr[0];
    mavlink_out_highres_imu_struct.ygyro = gyr[1];
    mavlink_out_highres_imu_struct.zgyro = gyr[2];
  }

  if (nullptr != mag){
    mavlink_out_highres_imu_struct.xmag = mag[0];
    mavlink_out_highres_imu_struct.ymag = mag[1];
    mavlink_out_highres_imu_struct.zmag = mag[2];
  }

  if ((nullptr != acc) || (nullptr != gyr) || (nullptr != mag)){
    //mavlink_out_highres_imu_struct.time_usec = TimeKeeper::utc();
  }
}

/**
 *
 */
static THD_WORKING_AREA(Mpu6050ThreadWA, 256);
static THD_FUNCTION(Mpu6050Thread, arg) {
  (void)arg;
  chRegSetThreadName("Mpu6050");

  while (!chThdShouldTerminateX()) {
    marg_sem.wait();

    mpu6050.get(acc_data, gyr_data);
    lsm303mag.get(mag_data);

    marg2mavlink(acc_data, gyr_data, mag_data);
  }

  chThdExit(MSG_OK);
  return 0;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void MargWorkerStart(void){
  chThdCreateStatic(Mpu6050ThreadWA, sizeof(Mpu6050ThreadWA),
                            NORMALPRIO, Mpu6050Thread, NULL);
  mpu6050.start();
  Exti.mpu6050(true);
  ak8975.start();
  lsm303mag.start();
  lsm303acc.start();
}

/**
 *
 */
void MPU6050ISR(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  osalSysLockFromISR();
  marg_sem.signalI();
  osalSysUnlockFromISR();
}
