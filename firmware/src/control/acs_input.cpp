#include "main.h"

#include "mavlink_local.hpp"
#include "acs_input.hpp"
#include "geometry.hpp"
#include "mav_mail.hpp"
#include "mav_logger.hpp"

using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern MavLogger mav_logger;

extern mavlink_global_position_int_t  mavlink_out_global_position_int_struct;
extern mavlink_attitude_t             mavlink_out_attitude_struct;
extern mavlink_attitude_quaternion_t  mavlink_out_attitude_quaternion_struct;

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

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
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
void acs_input2mavlink(const ACSInput &acs_in) {
  double tmp;

  tmp = acs_in.ch[ACS_INPUT_lat];
  mavlink_out_global_position_int_struct.lat = rad2deg(tmp) * DEG_TO_MAVLINK;
  tmp = acs_in.ch[ACS_INPUT_lon];
  mavlink_out_global_position_int_struct.lon = rad2deg(tmp) * DEG_TO_MAVLINK;
  mavlink_out_global_position_int_struct.alt = acs_in.ch[ACS_INPUT_alt] * 1000;
  mavlink_out_global_position_int_struct.hdg = acs_in.ch[ACS_INPUT_yaw];
  mavlink_out_global_position_int_struct.time_boot_ms = TIME_BOOT_MS;

  mavlink_out_attitude_struct.roll  = acs_in.ch[ACS_INPUT_roll];
  mavlink_out_attitude_struct.pitch = acs_in.ch[ACS_INPUT_pitch];
  mavlink_out_attitude_struct.yaw   = acs_in.ch[ACS_INPUT_yaw];
  mavlink_out_attitude_struct.time_boot_ms = TIME_BOOT_MS;

  mavlink_out_attitude_quaternion_struct.q1 = acs_in.ch[ACS_INPUT_q0];
  mavlink_out_attitude_quaternion_struct.q2 = acs_in.ch[ACS_INPUT_q1];
  mavlink_out_attitude_quaternion_struct.q3 = acs_in.ch[ACS_INPUT_q2];
  mavlink_out_attitude_quaternion_struct.q4 = acs_in.ch[ACS_INPUT_q3];
  mavlink_out_attitude_quaternion_struct.time_boot_ms = TIME_BOOT_MS;

  log_append();
}


