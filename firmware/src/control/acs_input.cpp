#include "main.h"

#include "mavlink_local.hpp"
#include "acs_input.hpp"
#include "geometry.hpp"
#include "mav_mail.hpp"
#include "mav_logger.hpp"
#include "gnss_receiver.hpp"

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
extern mavlink_vfr_hud_t              mavlink_out_vfr_hud_struct;

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
static mavMail global_position_int_mail;
static mavMail vfr_hud_mail;

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

  if (global_position_int_mail.free()) {
    global_position_int_mail.fill(&mavlink_out_global_position_int_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
    mav_logger.write(&global_position_int_mail);
  }

  if (vfr_hud_mail.free()) {
    vfr_hud_mail.fill(&mavlink_out_vfr_hud_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_VFR_HUD);
    mav_logger.write(&vfr_hud_mail);
  }
}

/**
 *
 */
void acs_input2mavlink(const ACSInput &acs_in) {

  mavlink_out_global_position_int_struct.lat = rad2deg(acs_in.ch[ACS_INPUT_lat]) * DEG_TO_MAVLINK;
  mavlink_out_global_position_int_struct.lon = rad2deg(acs_in.ch[ACS_INPUT_lon]) * DEG_TO_MAVLINK;
  mavlink_out_global_position_int_struct.alt = acs_in.ch[ACS_INPUT_alt] * 1000;
  mavlink_out_global_position_int_struct.hdg = acs_in.ch[ACS_INPUT_yaw];
  mavlink_out_global_position_int_struct.vx = round(100 * acs_in.ch[ACS_INPUT_vx]);
  mavlink_out_global_position_int_struct.vy = round(100 * acs_in.ch[ACS_INPUT_vy]);
  mavlink_out_global_position_int_struct.vz = round(100 * acs_in.ch[ACS_INPUT_vz]);
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

/**
 *
 */
void gps2acs_in(const gnss::gnss_data_t &gps, ACSInput &acs_in) {

  acs_in.ch[ACS_INPUT_lat] = gps.latitude;
  acs_in.ch[ACS_INPUT_lon] = gps.longitude;
  acs_in.ch[ACS_INPUT_alt] = gps.altitude;
}

/**
 *
 */
void baro2acs_in(const baro_data_t &baro, ACSInput &acs_in) {
  acs_in.ch[ACS_INPUT_alt_baro] = baro.alt;
}

/**
 *
 */
void speedometer2acs_in(const odometer_data_t &speed, ACSInput &acs_in) {
  acs_in.ch[ACS_INPUT_odo_speed] = speed.speed;
}


