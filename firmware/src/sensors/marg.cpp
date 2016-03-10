#include "main.h"

#include "mavlink_local.hpp"
#include "param_registry.hpp"
#include "mav_mail.hpp"
#include "mav_logger.hpp"
#include "time_keeper.hpp"
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

//extern mavlink_raw_imu_t                mavlink_out_raw_imu_struct;
extern mavlink_scaled_imu_t             mavlink_out_scaled_imu_struct;

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
//static mavMail raw_imu_mail;
static mavMail scaled_imu_mail;

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
  mavlink_out_scaled_imu_struct.xacc = data.acc[0] * 1000;
  mavlink_out_scaled_imu_struct.yacc = data.acc[1] * 1000;
  mavlink_out_scaled_imu_struct.zacc = data.acc[2] * 1000;

  mavlink_out_scaled_imu_struct.xgyro = data.gyr[0] * 1000;
  mavlink_out_scaled_imu_struct.ygyro = data.gyr[1] * 1000;
  mavlink_out_scaled_imu_struct.zgyro = data.gyr[2] * 1000;

  mavlink_out_scaled_imu_struct.xmag = data.mag[0] * 1000;
  mavlink_out_scaled_imu_struct.ymag = data.mag[1] * 1000;
  mavlink_out_scaled_imu_struct.zmag = data.mag[2] * 1000;

  mavlink_out_scaled_imu_struct.time_boot_ms = TIME_BOOT_MS;

  /*
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
  */
}

/**
 *
 */
static void log_append(void) {

  /*
  if (raw_imu_mail.free()) {
    raw_imu_mail.fill(&mavlink_out_raw_imu_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_RAW_IMU);
    mav_logger.write(&raw_imu_mail);
  }
  */

  if (scaled_imu_mail.free()) {
    scaled_imu_mail.fill(&mavlink_out_scaled_imu_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_SCALED_IMU);
    mav_logger.write(&scaled_imu_mail);
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
sensor_state_registry_t Marg::param_update(void) {

  uint8_t a, g, m;

  osalSysLock();
  a = *acc_src;
  g = *gyr_src;
  m = *mag_src;
  osalSysUnlock();

  if ((this->acc_src_current != a) || (this->gyr_src_current != g) || (this->mag_src_current != m)) {
    sleep_all();

    if (SENSOR_STATE_SLEEP == adis.get_state()) {
      if ((MARG_ACC_SRC_ADIS == a) || (MARG_GYR_SRC_ADIS == g) ||
                                      (MARG_MAG_SRC_ADIS == m)){
        adis.wakeup();
      }
    }
    if (SENSOR_STATE_SLEEP == mpu6050.get_state()) {
      if ((MARG_ACC_SRC_MPU6050 == a) || (MARG_GYR_SRC_MPU6050 == g)) {
        mpu6050.wakeup();
      }
    }
    if (SENSOR_STATE_SLEEP == ak8975.get_state()) {
      if (MARG_MAG_SRC_AK8975 == m){
        ak8975.wakeup();
      }
    }
    if (SENSOR_STATE_SLEEP == lsm303mag.get_state()) {
      if (MARG_MAG_SRC_LSM303 == m) {
        lsm303mag.wakeup();
      }
    }
    if (SENSOR_STATE_SLEEP == lsm303acc.get_state()) {
      if (MARG_ACC_SRC_LSM303 == a) {
        lsm303acc.wakeup();
      }
    }
  }

  this->acc_src_current = a;
  this->gyr_src_current = g;
  this->mag_src_current = m;

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
    mpu6050(&MPU6050_I2CD,  MPU6050_I2C_ADDR),
    ak8975(&MPU6050_I2CD,   AK8975_I2C_ADDR),
    lsm303mag(&LSM303_I2CD, LSM303_MAG_I2C_ADDR),
    lsm303acc(&LSM303_I2CD, LSM303_ACC_I2C_ADDR)
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
  this->acc_src_current = !*acc_src;
  this->gyr_src_current = !*gyr_src;
  this->mag_src_current = !*mag_src;

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

  ret = this->param_update();
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
  this->param_update();

  /*
   * wait data from "main" sensor
   */
  switch(this->gyr_src_current) {
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
   * start from clean state
   */
  memset(&result, 0, sizeof(result));

  /*
   * ADIS
   */
  memset(&result.request, 0, sizeof(result.request));
  if (MARG_ACC_SRC_ADIS == acc_src_current)
    result.request.acc = 1;
  if (MARG_MAG_SRC_ADIS == mag_src_current)
    result.request.mag = 1;
  if (MARG_GYR_SRC_ADIS == gyr_src_current) {
    result.request.gyr = 1;
    result.request.dT = 1;
  }
  adis.get(result);

  /*
   * MPU6050
   */
  memset(&result.request, 0, sizeof(result.request));
  if (MARG_ACC_SRC_MPU6050 == acc_src_current)
    result.request.acc = 1;
  if (MARG_GYR_SRC_MPU6050 == gyr_src_current) {
    result.request.gyr = 1;
    result.request.dT = 1;
  }
  mpu6050.get(result);

  /*
   * LSM303 accel
   */
  memset(&result.request, 0, sizeof(result.request));
  if (MARG_ACC_SRC_LSM303 == acc_src_current)
    result.request.acc = 1;
  lsm303acc.get(result);

  /*
   * LSM303 mag
   */
  memset(&result.request, 0, sizeof(result.request));
  if (MARG_MAG_SRC_LSM303 == mag_src_current)
    result.request.mag = 1;
  lsm303mag.get(result);

  /*
   * AK8975
   */
  memset(&result.request, 0, sizeof(result.request));
  if (MARG_MAG_SRC_AK8975 == mag_src_current)
    result.request.mag = 1;
  ak8975.get(result);

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
