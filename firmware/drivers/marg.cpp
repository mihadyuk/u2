#include "main.h"
#include "mpu6050.hpp"
#include "ak8975.hpp"
#include "lsm303_acc.hpp"
#include "lsm303_mag.hpp"
#include "mavlink_local.hpp"
#include "marg2mavlink.hpp"
#include "param_registry.hpp"

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

static BinarySemaphore adis_sem(true);
static BinarySemaphore mpu6050_sem(true);

//static MPU6050 mpu6050(&I2CD_FAST, mpu6050addr);
static AK8975 ak8975(&I2CD_FAST, ak8975addr);
static LSM303_mag lsm303mag(&I2CD_FAST, lsm303magaddr);
static LSM303_acc lsm303acc(&I2CD_FAST, lsm303accaddr);

//static float acc_data[3];
//static float gyr_data[3];
//static float mag_data[3];

static const uint32_t *gyr_src = NULL;
static const uint32_t *acc_src = NULL;
static const uint32_t *mag_src = NULL;

static const uint32_t *mpu_dlpf = NULL;
static const uint32_t *mpu_smplrt_div = NULL;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */



/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void MargStart(void) {

  param_registry.valueSearch("MPU_dlpf",      &mpu_dlpf);
  param_registry.valueSearch("MPU_smplrt_div",&mpu_smplrt_div);

  param_registry.valueSearch("MARG_acc_src",  &acc_src);
  param_registry.valueSearch("MARG_gyr_src",  &gyr_src);
  param_registry.valueSearch("MARG_mag_src",  &mag_src);

  ak8975.start();
  lsm303mag.start();
  lsm303acc.start();
}

/**
 *
 */
void MargStop(void) {

  ak8975.stop();
  lsm303mag.stop();
  lsm303acc.stop();
}




