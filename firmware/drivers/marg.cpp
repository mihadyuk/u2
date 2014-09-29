#include "main.h"
#include "exti_local.hpp"
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

static BinarySemaphore marg_sem(true);

static MPU6050 mpu6050(&I2CD_FAST, mpu6050addr);
static AK8975 ak8975(&I2CD_FAST, ak8975addr);
static LSM303_mag lsm303mag(&I2CD_FAST, lsm303magaddr);
static LSM303_acc lsm303acc(&I2CD_FAST, lsm303accaddr);

static float acc_data[3];
static float gyr_data[3];
static float mag_data[3];

static thread_t *worker = NULL;

static const uint32_t *gyr_src = NULL;
static const uint32_t *acc_src = NULL;
static const uint32_t *mag_src = NULL;

static const uint32_t *mpu_dlpf = NULL;
static const uint32_t *mpu_smplrt_div = NULL;
static uint32_t mpu_int_counter = 0;

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
static THD_WORKING_AREA(Mpu6050ThreadWA, 256);
static THD_FUNCTION(Mpu6050Thread, arg) {
  (void)arg;
  chRegSetThreadName("Mpu6050");

  while (!chThdShouldTerminateX()) {
    marg_sem.wait();

    /* accelerometer and gyroscope acquisition */
    mpu6050.get(acc_data, gyr_data);

    /* magnetic data acquisition */
    switch (*mag_src){
    case MARG_MAG_SRC_LSM303:
      lsm303mag.get(mag_data);
      break;
    case MARG_MAG_SRC_AK8975:
      ak8975.get(mag_data);
      break;
    default:
      osalSysHalt("Unhandled case");
      break;
    }

    /* mavlink message fill */
    marg2highres_imu(acc_data, gyr_data, mag_data);
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
void MargStart(void) {

  param_registry.valueSearch("MPU_dlpf",      &mpu_dlpf);
  param_registry.valueSearch("MPU_smplrt_div",&mpu_smplrt_div);

  param_registry.valueSearch("MARG_acc_src",  &acc_src);
  param_registry.valueSearch("MARG_gyr_src",  &gyr_src);
  param_registry.valueSearch("MARG_mag_src",  &mag_src);

  mpu6050.start();
  Exti.mpu6050(true);
  ak8975.start();
  lsm303mag.start();
  lsm303acc.start();

  worker = chThdCreateStatic(Mpu6050ThreadWA, sizeof(Mpu6050ThreadWA),
                             NORMALPRIO, Mpu6050Thread, NULL);
  osalDbgAssert(NULL != worker, "Can not allocate RAM");
}

/**
 *
 */
void MargStop(void) {

  Exti.mpu6050(false);

  chThdTerminate(worker);
  chThdWait(worker);

  mpu6050.stop();
  ak8975.stop();
  lsm303mag.stop();
  lsm303acc.stop();
}

/**
 *
 */
void MPU6050ISR(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  osalSysLockFromISR();

  if (0 == *mpu_dlpf){ /* we need software rate divider */
    mpu_int_counter++;
    if (mpu_int_counter >= *mpu_smplrt_div){
      mpu_int_counter = 0;
      marg_sem.signalI();
    }
  }
  else { /* signal semaphore every interrupt pulse */
    marg_sem.signalI();
  }

  osalSysUnlockFromISR();
}


