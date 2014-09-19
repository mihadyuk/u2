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
 * @brief   Magnetic measurement source
 */
typedef enum {
  MARG_MAG_SRC_NONE     = 0,
  MARG_MAG_SRC_LSM303   = 1,
  MARG_MAG_SRC_AK8975   = 2,
  MARG_MAG_SRC_ADIS     = 3,
} mag_src_t;

/**
 * @brief   Acceleration measurement source
 */
typedef enum {
  MARG_ACC_SRC_NONE     = 0,
  MARG_ACC_SRC_LSM303   = 1,
  MARG_ACC_SRC_MPU6050  = 2,
  MARG_ACC_SRC_ADIS     = 3,
} acc_src_t;

/**
 * @brief   Angular rate measurement source
 */
typedef enum {
  MARG_GYR_SRC_NONE     = 0,
  MARG_GYR_SRC_MPU6050  = 1,
  MARG_GYR_SRC_ADIS     = 2,
} gyr_src_t;

/**
 * @brief   Synchronization source
 */
typedef enum {
  MARG_SYNC_SRC_SOFT     = 0,
  MARG_SYNC_SRC_MPU6050  = 1,
  MARG_SYNC_SRC_ADIS     = 2,
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

static BinarySemaphore mpu6050_sem(true);
static MPU6050 mpu6050(&I2CD_FAST, mpu6050addr);
static AK8975 ak8975(&I2CD_FAST, ak8975addr);
static LSM303_mag lsm303mag(&I2CD_FAST, lsm303magaddr);

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
    mpu6050_sem.wait();

    mpu6050.get(acc_data, gyr_data);
    ak8975.get(mag_data);

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
void MemsWorkerStart(void){
  chThdCreateStatic(Mpu6050ThreadWA, sizeof(Mpu6050ThreadWA),
                            NORMALPRIO, Mpu6050Thread, NULL);
  mpu6050.start();
  ak8975.start();
  Exti.mpu6050_enable(true);
}

/**
 *
 */
void MPU6050ISR(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  osalSysLockFromISR();
  mpu6050_sem.signalI();
  osalSysUnlockFromISR();
}
