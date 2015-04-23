#include "main.h"

#include "ahrs.hpp"
#include "mavlink_local.hpp"
#include "param_registry.hpp"
#include "mav_mail.hpp"
#include "mav_logger.hpp"
#include "time_keeper.hpp"
#include "geometry.hpp"

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
static mavMail attitude_mail;
static mavMail attitude_quaternion_mail;

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
static void attitude2mavlink(const ahrs_data_t &att) {

  mavlink_out_attitude_struct.roll  = att.euler[0];
  mavlink_out_attitude_struct.pitch = att.euler[1];
  mavlink_out_attitude_struct.yaw   = att.euler[2];
  mavlink_out_attitude_struct.time_boot_ms = TIME_BOOT_MS;

  mavlink_out_attitude_quaternion_struct.q1 = att.quat[0];
  mavlink_out_attitude_quaternion_struct.q2 = att.quat[1];
  mavlink_out_attitude_quaternion_struct.q3 = att.quat[2];
  mavlink_out_attitude_quaternion_struct.q4 = att.quat[3];
  mavlink_out_attitude_quaternion_struct.time_boot_ms = TIME_BOOT_MS;
}

/**
 *
 */
static void attitude2state_vector(const ahrs_data_t &att, StateVector &sv) {
  sv.ch[STATE_VECTOR_roll]  = att.euler[0];
  sv.ch[STATE_VECTOR_pitch] = att.euler[1];
  sv.ch[STATE_VECTOR_yaw]   = att.euler[2];

  sv.ch[STATE_VECTOR_q0] = att.quat[0];
  sv.ch[STATE_VECTOR_q1] = att.quat[1];
  sv.ch[STATE_VECTOR_q2] = att.quat[2];
  sv.ch[STATE_VECTOR_q3] = att.quat[3];
}

/**
 *
 */
static void log_append(void) {

  if (attitude_mail.free()) {
    attitude_mail.fill(&mavlink_out_attitude_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_ATTITUDE);
    mav_logger.write(&attitude_mail);
  }

  if (attitude_quaternion_mail.free()) {
    attitude_quaternion_mail.fill(&mavlink_out_attitude_quaternion_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_ATTITUDE_QUATERNION);
    mav_logger.write(&attitude_quaternion_mail);
  }
}

/**
 *
 */
msg_t Ahrs::get_starlino(ahrs_data_t &result, systime_t timeout) {
  (void)result;

  marg_data_t marg_data;
  msg_t sem_status = MSG_RESET;

  sem_status = this->marg.get(marg_data, timeout);
  if (MSG_OK == sem_status) {
    result.dt = marg_data.dt;
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
    result.dt = marg_data.dt;
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
    result.dt = marg_data.dt;
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
void Ahrs::reschedule(void) {
  uint8_t m;

  osalSysLock();
  m = *mode;
  osalSysUnlock();

  if (this->mode_current != m) {
    if (AHRS_MODE_ADIS == m) {
      marg.stop();
      adis.start();
    }
    else {
      adis.stop();
      marg.start();
    }
  }

  this->mode_current = m;
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
  mode_current = *mode;

  if (AHRS_MODE_ADIS == mode_current)
    adis.start();
  else
    marg.start();

  state = AHRS_STATE_READY;
}

/**
 *
 */
msg_t Ahrs::get(ahrs_data_t &result, StateVector &sv, systime_t timeout) {
  msg_t sem_status = MSG_RESET;

  reschedule();

  switch(mode_current) {
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

  attitude2mavlink(result);
  attitude2state_vector(result, sv);
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


