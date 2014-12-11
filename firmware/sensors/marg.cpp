#include "main.h"

#include "mavlink_local.hpp"
#include "param_registry.hpp"
#include "mav_mail.hpp"
#include "mav_logger.hpp"
#include "timekeeper.hpp"
#include "marg.hpp"

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
extern sensor_state_registry_t SensorStateRegistry;
extern MavLogger mav_logger;

extern mavlink_raw_imu_t                mavlink_out_raw_imu_struct;
extern mavlink_highres_imu_t            mavlink_out_highres_imu_struct;

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
void marg2mavlink(const marg_data_t &data) {

  /**/
  mavlink_out_highres_imu_struct.xacc = data.acc[0];
  mavlink_out_highres_imu_struct.yacc = data.acc[1];
  mavlink_out_highres_imu_struct.zacc = data.acc[2];

  mavlink_out_highres_imu_struct.xgyro = data.gyr[0];
  mavlink_out_highres_imu_struct.ygyro = data.gyr[1];
  mavlink_out_highres_imu_struct.zgyro = data.gyr[2];

  mavlink_out_highres_imu_struct.xmag = data.mag[0];
  mavlink_out_highres_imu_struct.ymag = data.mag[1];
  mavlink_out_highres_imu_struct.zmag = data.mag[2];

  mavlink_out_highres_imu_struct.time_usec = TimeKeeper::utc();

  /**/
  mavlink_out_raw_imu_struct.xacc = data.acc_raw[0];
  mavlink_out_raw_imu_struct.yacc = data.acc_raw[1];
  mavlink_out_raw_imu_struct.zacc = data.acc_raw[2];

  mavlink_out_raw_imu_struct.xgyro = data.gyr_raw[0];
  mavlink_out_raw_imu_struct.ygyro = data.gyr_raw[1];
  mavlink_out_raw_imu_struct.zgyro = data.gyr_raw[2];

  mavlink_out_raw_imu_struct.xmag = data.mag_raw[0];
  mavlink_out_raw_imu_struct.ymag = data.mag_raw[1];
  mavlink_out_raw_imu_struct.zmag = data.mag_raw[2];

  mavlink_out_raw_imu_struct.time_usec = TimeKeeper::utc();
}

/**
 *
 */
static void log_append(void) {

  if (raw_imu_mail.free()) {
    raw_imu_mail.fill(&mavlink_out_raw_imu_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_RAW_IMU);
    mav_logger.post(&raw_imu_mail);
  }

  if (highres_imu_mail.free()) {
    highres_imu_mail.fill(&mavlink_out_highres_imu_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_HIGHRES_IMU);
    mav_logger.post(&highres_imu_mail);
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
sensor_state_registry_t Marg::reschedule(void) {

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
Marg::Marg(Adis &adis) :
    adis(adis),
    mpu6050(&I2CD_FAST, mpu6050addr),
    ak8975(&I2CD_FAST, ak8975addr),
    lsm303mag(&I2CD_FAST, lsm303magaddr),
    lsm303acc(&I2CD_FAST, lsm303accaddr)
{
  return;
}

/**
 *
 */
sensor_state_registry_t Marg::start(void) {

  sensor_state_registry_t ret;

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
msg_t Marg::get(marg_data_t &result, systime_t timeout) {
  msg_t sem_status = MSG_RESET;

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
    sem_status = mpu6050.waitData(timeout);
    break;
  case MARG_GYR_SRC_ADIS:
    sem_status = adis.waitData(timeout);
    break;
  default:
    osalSysHalt("Unhandled case");
    break;
  }
  if (MSG_OK != sem_status)
    return sem_status;

  /*
   * check ADIS
   */
  memset(&result.request, 0, sizeof(result.request));
  if (MARG_ACC_SRC_ADIS == *acc_src)
    result.request.acc = 1;
  if (MARG_MAG_SRC_ADIS == *mag_src)
    result.request.mag = 1;
  if (MARG_GYR_SRC_ADIS == *gyr_src) {
    result.request.gyr = 1;
    result.request.dt = 1;
  }
  adis.get(result);

  /*
   * check MPU6050
   */
  memset(&result.request, 0, sizeof(result.request));
  if (MARG_ACC_SRC_MPU6050 == *acc_src)
    result.request.acc = 1;
  if (MARG_GYR_SRC_MPU6050 == *gyr_src) {
    result.request.gyr = 1;
    result.request.dt = 1;
  }
  mpu6050.get(result);

  /*
   * LSM303 accel
   */
  memset(&result.request, 0, sizeof(result.request));
  if (MARG_ACC_SRC_LSM303 == *acc_src)
    result.request.acc = 1;
  lsm303acc.get(result);

  /*
   * LSM303 mag
   */
  memset(&result.request, 0, sizeof(result.request));
  if (MARG_MAG_SRC_LSM303 == *mag_src)
    result.request.mag = 1;
  lsm303mag.get(result);

  /*
   * AK8975
   */
  memset(&result.request, 0, sizeof(result.request));
  if (MARG_MAG_SRC_AK8975 == *mag_src)
    result.request.mag = 1;
  ak8975.get(result);


  /*
   * now collect data
   */
//  if ((MARG_ACC_SRC_ADIS == *acc_src) || (MARG_GYR_SRC_ADIS == *gyr_src) ||
//                                         (MARG_MAG_SRC_ADIS == *mag_src))
//    ret->reg.adis = adis.get(&adis_marg);
//  if ((MARG_ACC_SRC_MPU6050 == *acc_src) || (MARG_GYR_SRC_MPU6050 == *gyr_src)) {
//    ret->reg.mpu6050 = mpu6050.get(mpu_acc, mpu_gyr, mpu_acc_raw, mpu_gyr_raw);
//  }
//  if (MARG_ACC_SRC_LSM303 == *acc_src) {
//    ret->reg.lsm303acc = lsm303acc.get(lsm_acc, lsm_acc_raw);
//  }
//  if (MARG_MAG_SRC_LSM303 == *mag_src) {
//    ret->reg.lsm303mag = lsm303mag.get(lsm_mag, lsm_mag_raw);
//  }
//  if (MARG_MAG_SRC_AK8975 == *mag_src) {
//    ret->reg.ak8975 = ak8975.get(ak_mag, ak_mag_raw);
//  }

//  /*
//   * now pick what we really need
//   */
//  switch(*this->acc_src) {
//  case MARG_ACC_SRC_MPU6050:
//    memcpy(ret->acc, mpu_acc, sizeof(mpu_acc));
//    acc2raw_imu(mpu_acc_raw);
//    break;
//  case MARG_ACC_SRC_ADIS:
//    memcpy(ret->acc, adis_marg.acc, sizeof(adis_marg.acc));
//    /* there is no raw data from adis currently */
//    break;
//  case MARG_ACC_SRC_LSM303:
//    memcpy(ret->acc, lsm_acc, sizeof(lsm_acc));
//    acc2raw_imu(lsm_acc_raw);
//    break;
//  default:
//    osalSysHalt("Unhandled case");
//    break;
//  }
//
//  switch(*this->gyr_src) {
//  case MARG_GYR_SRC_MPU6050:
//    memcpy(ret->gyr, mpu_gyr, sizeof(mpu_gyr));
//    ret->dt = mpu6050.dt();
//    gyr2raw_imu(mpu_gyr_raw);
//    break;
//  case MARG_GYR_SRC_ADIS:
//    memcpy(ret->gyr, adis_marg.gyr, sizeof(adis_marg.gyr));
//    ret->dt = adis.dt();
//    /* there is no raw data from adis currently */
//    break;
//  default:
//    osalSysHalt("Unhandled case");
//    break;
//  }
//
//  switch(*this->mag_src) {
//  case MARG_MAG_SRC_ADIS:
//    memcpy(ret->mag, adis_marg.mag, sizeof(adis_marg.mag));
//    /* there is no raw data from adis currently */
//    break;
//  case MARG_MAG_SRC_AK8975:
//    memcpy(ret->mag, ak_mag, sizeof(ak_mag));
//    mag2raw_imu(ak_mag_raw);
//    break;
//  case MARG_MAG_SRC_LSM303:
//    memcpy(ret->mag, lsm_mag, sizeof(lsm_mag));
//    mag2raw_imu(lsm_mag_raw);
//    break;
//  default:
//    osalSysHalt("Unhandled case");
//    break;
//  }

  /*
   * fill debug structure
   */
  marg2mavlink(result);
  SensorStateRegistry = get_state();
  log_append();
  return sem_status;
}

/**
 *
 */
sensor_state_registry_t Marg::get_state(void) {
  sensor_state_registry_t ret;

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