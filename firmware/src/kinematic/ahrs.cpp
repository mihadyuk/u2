#include "main.h"

#include "ahrs.hpp"
#include "mavlink_local.hpp"
#include "param_registry.hpp"
#include "mav_mail.hpp"
#include "mav_logger.hpp"
#include "timekeeper.hpp"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
/**
 *
 */
typedef enum {
  AHRS_MODE_STARLINO = 0,
  AHRS_MODE_MADGWICK,
  AHRS_MODE_ADIS,
  AHRS_MODE_KALMAN,
}ahrs_mode_t;

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern MavLogger mav_logger;

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
msg_t Ahrs::get_starlino(ahrs_data_t &result, systime_t timeout) {
  (void)result;

  marg_data_t marg_data;
  msg_t sem_status = MSG_RESET;

  sem_status = this->marg.get(marg_data, timeout);
  if (MSG_OK == sem_status) {
    ; // TODO: process data here
  }

  return MSG_RESET;
}

/**
 *
 */
msg_t Ahrs::get_madgwick(ahrs_data_t &result, systime_t timeout) {
  (void)result;

  marg_data_t marg_data;
  msg_t sem_status = MSG_RESET;

  sem_status = this->marg.get(marg_data, timeout);
  if (MSG_OK == sem_status) {
    ; // TODO: process data here
  }

  return MSG_RESET;
}

/**
 *
 */
msg_t Ahrs::get_kalman(ahrs_data_t &result, systime_t timeout) {
  (void)result;

  marg_data_t marg_data;
  msg_t sem_status = MSG_RESET;

  sem_status = this->marg.get(marg_data, timeout);
  if (MSG_OK == sem_status) {
    ; // TODO: process data here
  }

  return MSG_RESET;
}

/**
 *
 */
msg_t Ahrs::get_adis(ahrs_data_t &result, systime_t timeout) {
  msg_t sem_status = MSG_RESET;

  sem_status = this->adis.waitData(timeout);
  if (MSG_OK == sem_status) {
    memset(&result.request, 0, sizeof(result.request));
    result.request.euler = 1;
    result.request.quat = 1;
    adis.get(result);
  }

  return sem_status;
}

/**
 *
 */
static void log_append(void) {

//  if (raw_imu_mail.free()) {
//    raw_imu_mail.fill(&mavlink_out_raw_imu_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_RAW_IMU);
//    mav_logger.post(&raw_imu_mail);
//  }
//
//  if (highres_imu_mail.free()) {
//    highres_imu_mail.fill(&mavlink_out_highres_imu_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_HIGHRES_IMU);
//    mav_logger.post(&highres_imu_mail);
//  }
}

/**
 *
 */
void Ahrs::reschedule(void) {

  uint8_t m;

  osalSysLock();
  m = *mode;
  osalSysUnlock();

  if (this->mode_prev != m) {
    ;
  }

  this->mode_prev = m;
}



/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
Ahrs::Ahrs(void) :
marg(adis)
{
  state = AHRS_STATE_STOP;
  return;
}

/**
 *
 */
void Ahrs::start(void) {

  param_registry.valueSearch("AHRS_mode", &mode);

  if (AHRS_MODE_ADIS == *mode)
    adis.start();
  else
    marg.start();

  state = AHRS_STATE_READY;
}

/**
 *
 */
msg_t Ahrs::get(ahrs_data_t &result, systime_t timeout) {
  msg_t sem_status = MSG_RESET;

  reschedule();

  switch(*mode) {
  case AHRS_MODE_STARLINO:
    sem_status = get_starlino(result, timeout);
    break;
  case AHRS_MODE_MADGWICK:
    sem_status = get_madgwick(result, timeout);
    break;
  case AHRS_MODE_ADIS:
    sem_status = get_adis(result, timeout);
    break;
  case AHRS_MODE_KALMAN:
    sem_status = get_kalman(result, timeout);
    break;
  default:
    osalSysHalt("Unhandled case");
    break;
  }

  log_append();
  return sem_status;
}

/**
 *
 */
void Ahrs::stop(void) {

  this->state = AHRS_STATE_STOP;
  marg.stop();
  adis.stop();
}


