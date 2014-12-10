#include "main.h"

#include "marg.hpp"
#include "mavlink_local.hpp"
#include "marg2mavlink.hpp"
#include "param_registry.hpp"
#include "mav_mail.hpp"
#include "mav_logger.hpp"

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
  MARG_MAG_SRC_ADIS,
  MARG_MAG_SRC_AK8975,
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

extern MavLogger mav_logger;

extern mavlink_raw_imu_t                mavlink_out_raw_imu_struct;
extern mavlink_highres_imu_t            mavlink_out_highres_imu_struct;
extern mavlink_attitude_t               mavlink_out_attitude_struct;
extern mavlink_attitude_quaternion_t    mavlink_out_attitude_quaternion_struct;

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

static mavMail raw_imu_mail;
static mavMail highres_imu_mail;
static mavMail attitude_imu_mail;
static mavMail attitude_quaternion_imu_mail;

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
static void log_append(void) {

  msg_t status = MSG_RESET;

  if (raw_imu_mail.free()) {
    raw_imu_mail.fill(&mavlink_out_raw_imu_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_RAW_IMU);
    status = mav_logger.post(&raw_imu_mail);
    if (MSG_OK != status)
      raw_imu_mail.release();
  }

  if (highres_imu_mail.free()) {
    highres_imu_mail.fill(&mavlink_out_highres_imu_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_HIGHRES_IMU);
    status = mav_logger.post(&highres_imu_mail);
    if (MSG_OK != status)
      raw_imu_mail.release();
  }
}

/**
 *
 */
void Marg::sleep_all(void) {
  if (SENSOR_STATE_READY == adis.get_state())
    adis.sleep();
  if (SENSOR_STATE_READY == mpu6050.get_state())
    mpu6050.sleep();
  if (SENSOR_STATE_READY == ak8975.get_state())
    ak8975.sleep();
  if (SENSOR_STATE_READY == lsm303mag.get_state())
    lsm303mag.sleep();
  if (SENSOR_STATE_READY == lsm303acc.get_state())
    lsm303acc.sleep();
}

/**
 *
 */
marg_state_reg_t Marg::reschedule(void) {

  uint8_t a, g, m;

  osalSysLock();
  a = *acc_src;
  g = *gyr_src;
  m = *mag_src;
  osalSysUnlock();

  if ((this->acc_src_prev != a) || (this->gyr_src_prev != g) || (this->mag_src_prev != m)) {
    sleep_all();

    if (SENSOR_STATE_SLEEP == adis.get_state()) {
      if ((MARG_ACC_SRC_ADIS == *acc_src) || (MARG_GYR_SRC_ADIS == *gyr_src) ||
                                             (MARG_MAG_SRC_ADIS == *mag_src)){
        adis.wakeup();
      }
    }
    if (SENSOR_STATE_SLEEP == mpu6050.get_state()) {
      if ((MARG_ACC_SRC_MPU6050 == *acc_src) || (MARG_GYR_SRC_MPU6050 == *gyr_src)) {
        mpu6050.wakeup();
      }
    }
    if (SENSOR_STATE_SLEEP == ak8975.get_state()) {
      if (MARG_MAG_SRC_AK8975 == *mag_src){
        ak8975.wakeup();
      }
    }
    if (SENSOR_STATE_SLEEP == lsm303mag.get_state()) {
      if (MARG_MAG_SRC_LSM303 == *mag_src) {
        lsm303mag.wakeup();
      }
    }
    if (SENSOR_STATE_SLEEP == lsm303acc.get_state()) {
      if (MARG_ACC_SRC_LSM303 == *acc_src) {
        lsm303acc.wakeup();
      }
    }
  }

  this->acc_src_prev = a;
  this->gyr_src_prev = g;
  this->mag_src_prev = m;

  return this->get_state();
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
Marg::Marg(void) :
    adis_sem(true),
    mpu6050_sem(true),
    adis(this->adis_sem),
    mpu6050(&I2CD_FAST, mpu6050addr, this->mpu6050_sem),
    ak8975(&I2CD_FAST, ak8975addr),
    lsm303mag(&I2CD_FAST, lsm303magaddr),
    lsm303acc(&I2CD_FAST, lsm303accaddr)
{
  return;
}

/**
 *
 */
marg_state_reg_t Marg::start(void) {

  marg_state_reg_t ret;

  param_registry.valueSearch("MARG_acc_src", &acc_src);
  param_registry.valueSearch("MARG_gyr_src", &gyr_src);
  param_registry.valueSearch("MARG_mag_src", &mag_src);

  /* enforce rescheduling */
  this->acc_src_prev = !*acc_src;
  this->gyr_src_prev = !*gyr_src;
  this->mag_src_prev = !*mag_src;

  /* start all sensors and put them in sleep state */
  if (SENSOR_STATE_READY == adis.start())
    adis.sleep();
  if (SENSOR_STATE_READY == mpu6050.start())
    mpu6050.sleep();
  if (SENSOR_STATE_READY == ak8975.start())
    ak8975.sleep();
  if (SENSOR_STATE_READY == lsm303mag.start())
    lsm303mag.sleep();
  if (SENSOR_STATE_READY == lsm303acc.start())
    lsm303acc.sleep();

  ret = this->reschedule();
  this->ready = true;
  return ret;
}

/**
 *
 */
msg_t Marg::update(marg_data_t *ret, systime_t timeout) {
  msg_t sem_status = MSG_RESET;
  adis_data_t adis_data;
  float mpu_acc[3], mpu_gyr[3];
  float lsm_acc[3], lsm_mag[3];
  float ak_mag[3];
  int16_t mpu_acc_raw[3], mpu_gyr_raw[3];
  int16_t lsm_acc_raw[3], lsm_mag_raw[3];
  int16_t ak_mag_raw[3];

  osalDbgCheck(true == this->ready);

  /*
   * sensor reschedule must be performed _before_ semaphore wait() call
   */
  this->reschedule();

  /*
   * wait data from "main" sensor
   */
  switch(*this->gyr_src) {
  case MARG_GYR_SRC_MPU6050:
    sem_status = this->mpu6050_sem.wait(timeout);
    break;
  case MARG_GYR_SRC_ADIS:
    sem_status = this->adis_sem.wait(timeout);
    break;
  default:
    osalSysHalt("Unhandled case");
    break;
  }
  if (MSG_OK != sem_status)
    return sem_status;

  /*
   * now collect data
   */
  if ((MARG_ACC_SRC_ADIS == *acc_src) || (MARG_GYR_SRC_ADIS == *gyr_src) ||
                                         (MARG_MAG_SRC_ADIS == *mag_src))
    ret->reg.adis = adis.get(&adis_data);
  if ((MARG_ACC_SRC_MPU6050 == *acc_src) || (MARG_GYR_SRC_MPU6050 == *gyr_src)) {
    ret->reg.mpu6050 = mpu6050.get(mpu_acc, mpu_gyr, mpu_acc_raw, mpu_gyr_raw);
  }
  if (MARG_ACC_SRC_LSM303 == *acc_src) {
    ret->reg.lsm303acc = lsm303acc.get(lsm_acc, lsm_acc_raw);
  }
  if (MARG_MAG_SRC_LSM303 == *mag_src) {
    ret->reg.lsm303mag = lsm303mag.get(lsm_mag, lsm_mag_raw);
  }
  if (MARG_MAG_SRC_AK8975 == *mag_src) {
    ret->reg.ak8975 = ak8975.get(ak_mag, ak_mag_raw);
  }

  /*
   * now pick what we really need
   */
  switch(*this->acc_src) {
  case MARG_ACC_SRC_MPU6050:
    memcpy(ret->acc, mpu_acc, sizeof(mpu_acc));
    acc2raw_imu(mpu_acc_raw);
    break;
  case MARG_ACC_SRC_ADIS:
    memcpy(ret->acc, adis_data.acc, sizeof(adis_data.acc));
    /* there is no raw data from adis currently */
    break;
  case MARG_ACC_SRC_LSM303:
    memcpy(ret->acc, lsm_acc, sizeof(lsm_acc));
    acc2raw_imu(lsm_acc_raw);
    break;
  default:
    osalSysHalt("Unhandled case");
    break;
  }

  switch(*this->gyr_src) {
  case MARG_GYR_SRC_MPU6050:
    memcpy(ret->gyr, mpu_gyr, sizeof(mpu_gyr));
    ret->dt = mpu6050.dt();
    gyr2raw_imu(mpu_gyr_raw);
    break;
  case MARG_GYR_SRC_ADIS:
    memcpy(ret->gyr, adis_data.gyr, sizeof(adis_data.gyr));
    ret->dt = adis.dt();
    /* there is no raw data from adis currently */
    break;
  default:
    osalSysHalt("Unhandled case");
    break;
  }

  switch(*this->mag_src) {
  case MARG_MAG_SRC_ADIS:
    memcpy(ret->mag, adis_data.mag, sizeof(adis_data.mag));
    /* there is no raw data from adis currently */
    break;
  case MARG_MAG_SRC_AK8975:
    memcpy(ret->mag, ak_mag, sizeof(ak_mag));
    mag2raw_imu(ak_mag_raw);
    break;
  case MARG_MAG_SRC_LSM303:
    memcpy(ret->mag, lsm_mag, sizeof(lsm_mag));
    mag2raw_imu(lsm_mag_raw);
    break;
  default:
    osalSysHalt("Unhandled case");
    break;
  }

  /*
   * fill debug structure
   */
  marg2highres_imu(ret->acc, ret->gyr, ret->mag);
  ret->reg = get_state();
  log_append();
  return sem_status;
}

/**
 *
 */
marg_state_reg_t Marg::get_state(void) {
  marg_state_reg_t ret;

  ret.adis      = adis.get_state();
  ret.mpu6050   = mpu6050.get_state();
  ret.ak8975    = ak8975.get_state();
  ret.lsm303mag = lsm303mag.get_state();
  ret.lsm303acc = lsm303acc.get_state();

  return ret;
}

/**
 *
 */
void Marg::stop(void) {

  this->ready = false;

  adis.stop();
  mpu6050.stop();
  ak8975.stop();
  lsm303mag.stop();
  lsm303acc.stop();
}





