#include <drivetrain_impact.hpp>
#include "main.h"
#include "message.hpp"
#include "acs_telemetry.hpp"
#include "servo_numbers.h"
#include "geometry.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define MASK_OUT_FLAGS      FALSE

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_rc_channels_raw_t      mavlink_out_rc_channels_raw_struct;
extern mavlink_rc_channels_scaled_t   mavlink_out_rc_channels_scaled_struct;
extern mavlink_vfr_hud_t              mavlink_out_vfr_hud_struct;
#if defined(BOARD_NTLAB_GRIFFON_ACS)
extern const mavlink_griffon_target_vector_t mavlink_out_griffon_target_vector_struct;
extern mavlink_hil_state_t            mavlink_out_hil_state_struct;
extern mavlink_griffon_state_vector_t mavlink_out_griffon_state_vector_struct;
extern mavlink_griffon_pid_dbg_t      mavlink_out_griffon_pid_dbg_struct;
extern mavlink_griffon_state_vector_t mavlink_out_griffon_state_vector_struct;
extern mavlink_griffon_volz_t         mavlink_out_griffon_volz_struct;
extern mavlink_griffon_558_plant_data_t mavlink_out_griffon_558_plant_data_struct;
extern mavlink_griffon_surface_angles_t mavlink_out_griffon_surface_angles_struct;
#endif /* defined(BOARD_NTLAB_GRIFFON_ACS) */

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
void pwm2telemetry(const PwmVector &pwm){

  mavlink_out_rc_channels_raw_struct.time_boot_ms = TIME_BOOT_MS;
  #if MASK_OUT_FLAGS
  const uint16_t VALUE_PWM_MASK = 0x0FFF;
  mavlink_out_rc_channels_raw_struct.chan1_raw = pwm.v[0] & VALUE_PWM_MASK;
  mavlink_out_rc_channels_raw_struct.chan2_raw = pwm.v[1] & VALUE_PWM_MASK;
  mavlink_out_rc_channels_raw_struct.chan3_raw = pwm.v[2] & VALUE_PWM_MASK;
  mavlink_out_rc_channels_raw_struct.chan4_raw = pwm.v[3] & VALUE_PWM_MASK;
  mavlink_out_rc_channels_raw_struct.chan5_raw = pwm.v[4] & VALUE_PWM_MASK;
  mavlink_out_rc_channels_raw_struct.chan6_raw = pwm.v[5] & VALUE_PWM_MASK;
  mavlink_out_rc_channels_raw_struct.chan7_raw = pwm.v[6] & VALUE_PWM_MASK;
  mavlink_out_rc_channels_raw_struct.chan8_raw = pwm.v[7] & VALUE_PWM_MASK;
  #else
  mavlink_out_rc_channels_raw_struct.chan1_raw = pwm.v[0];
  mavlink_out_rc_channels_raw_struct.chan2_raw = pwm.v[1];
  mavlink_out_rc_channels_raw_struct.chan3_raw = pwm.v[2];
  mavlink_out_rc_channels_raw_struct.chan4_raw = pwm.v[3];
  mavlink_out_rc_channels_raw_struct.chan5_raw = pwm.v[4];
  mavlink_out_rc_channels_raw_struct.chan6_raw = pwm.v[5];
  mavlink_out_rc_channels_raw_struct.chan7_raw = pwm.v[6];
  mavlink_out_rc_channels_raw_struct.chan8_raw = pwm.v[7];
  #endif /* MASK_OUT_FLAGS */

#if defined(BOARD_NTLAB_GRIFFON_ACS)
  mavlink_out_griffon_volz_struct.futaba00 = pwm.v[0];
  mavlink_out_griffon_volz_struct.futaba01 = pwm.v[1];
  mavlink_out_griffon_volz_struct.futaba02 = pwm.v[2];
  mavlink_out_griffon_volz_struct.futaba03 = pwm.v[3];
  mavlink_out_griffon_volz_struct.futaba04 = pwm.v[4];
  mavlink_out_griffon_volz_struct.futaba05 = pwm.v[5];
  mavlink_out_griffon_volz_struct.futaba06 = pwm.v[6];
  mavlink_out_griffon_volz_struct.futaba07 = pwm.v[7];
  mavlink_out_griffon_volz_struct.futaba08 = pwm.v[8];
  mavlink_out_griffon_volz_struct.futaba09 = pwm.v[9];
  mavlink_out_griffon_volz_struct.futaba10 = pwm.v[10];
  mavlink_out_griffon_volz_struct.futaba11 = pwm.v[11];
  mavlink_out_griffon_volz_struct.futaba12 = pwm.v[12];
  mavlink_out_griffon_volz_struct.futaba13 = pwm.v[13];

  mavlink_out_griffon_558_plant_data_struct.futaba00 = pwm.v[0];
  mavlink_out_griffon_558_plant_data_struct.futaba01 = pwm.v[1];
  mavlink_out_griffon_558_plant_data_struct.futaba02 = pwm.v[2];
  mavlink_out_griffon_558_plant_data_struct.futaba03 = pwm.v[3];
  mavlink_out_griffon_558_plant_data_struct.futaba04 = pwm.v[4];
  mavlink_out_griffon_558_plant_data_struct.futaba05 = pwm.v[5];
  mavlink_out_griffon_558_plant_data_struct.futaba06 = pwm.v[6];
  mavlink_out_griffon_558_plant_data_struct.futaba07 = pwm.v[7];
#endif
}

/**
 *
 */
void impact2telemetry(const Impact &impact){

  mavlink_out_rc_channels_scaled_struct.time_boot_ms = TIME_BOOT_MS;
  mavlink_out_rc_channels_scaled_struct.chan1_scaled = impact.angle[0] * 1000;
  mavlink_out_rc_channels_scaled_struct.chan2_scaled = impact.angle[1] * 1000;
  mavlink_out_rc_channels_scaled_struct.chan3_scaled = impact.angle[2] * 1000;
  mavlink_out_rc_channels_scaled_struct.chan4_scaled = impact.angle[3] * 1000;
  mavlink_out_rc_channels_scaled_struct.chan5_scaled = impact.angle[4] * 1000;
  mavlink_out_rc_channels_scaled_struct.chan6_scaled = impact.angle[5] * 1000;
  mavlink_out_rc_channels_scaled_struct.chan7_scaled = impact.angle[6] * 1000;
  //mavlink_out_rc_channels_scaled_struct.chan8_scaled = impact.angle[7] * 1000;
  mavlink_out_rc_channels_scaled_struct.chan8_scaled = impact.angle[SERVO_NUMBER_OTHER_BREAK] * 1000;

#if defined(BOARD_NTLAB_GRIFFON_ACS)
  mavlink_out_griffon_volz_struct.impact00 = impact.angle[0]  * 1000;
  mavlink_out_griffon_volz_struct.impact01 = impact.angle[1]  * 1000;
  mavlink_out_griffon_volz_struct.impact02 = impact.angle[2]  * 1000;
  mavlink_out_griffon_volz_struct.impact03 = impact.angle[3]  * 1000;
  mavlink_out_griffon_volz_struct.impact04 = impact.angle[4]  * 1000;
  mavlink_out_griffon_volz_struct.impact05 = impact.angle[5]  * 1000;
  mavlink_out_griffon_volz_struct.impact06 = impact.angle[6]  * 1000;
  mavlink_out_griffon_volz_struct.impact07 = impact.angle[7]  * 1000;
  mavlink_out_griffon_volz_struct.impact08 = impact.angle[8]  * 1000;
  mavlink_out_griffon_volz_struct.impact09 = impact.angle[9]  * 1000;
  mavlink_out_griffon_volz_struct.impact10 = impact.angle[10] * 1000;
  mavlink_out_griffon_volz_struct.impact11 = impact.angle[11] * 1000;
  mavlink_out_griffon_volz_struct.impact12 = impact.angle[12] * 1000;
  mavlink_out_griffon_volz_struct.impact13 = impact.angle[13] * 1000;

  //mavlink_out_vfr_hud_struct.throttle = mavlink_out_griffon_state_vector_struct.engine_rpm;
  mavlink_out_vfr_hud_struct.throttle = mavlink_out_griffon_state_vector_struct.engine_throttle * 10;
#endif
}

/**
 *
 */
#if defined(BOARD_NTLAB_GRIFFON_ACS)
void stabilizer2telemetry(const PIDControlNG<float> &pid_spd,
                          const PIDControlNG<float> &pid_ail,
                          const PIDControlNG<float> &pid_ele,
                          const PIDControlNG<float> &pid_rud,
                          float roll_impact_pid, float pitch_impact_pid){

  mavlink_out_griffon_pid_dbg_struct.pid_spd_i = pid_spd.dbg_getiState();
  mavlink_out_griffon_pid_dbg_struct.pid_ail_i = pid_ail.dbg_getiState();
  mavlink_out_griffon_pid_dbg_struct.pid_ele_i = pid_ele.dbg_getiState();
  mavlink_out_griffon_pid_dbg_struct.pid_rud_i = pid_rud.dbg_getiState();

  mavlink_out_hil_state_struct.lat    = rad2deg(mavlink_out_griffon_target_vector_struct.lat) * 10000000;
  mavlink_out_hil_state_struct.lon    = rad2deg(mavlink_out_griffon_target_vector_struct.lon) * 10000000;
  mavlink_out_hil_state_struct.alt    = mavlink_out_griffon_target_vector_struct.alt * 1000;

  mavlink_out_hil_state_struct.roll   = mavlink_out_griffon_target_vector_struct.roll;
  mavlink_out_hil_state_struct.pitch  = mavlink_out_griffon_target_vector_struct.pitch;
  mavlink_out_hil_state_struct.yaw    = mavlink_out_griffon_target_vector_struct.yaw;

  mavlink_out_hil_state_struct.pitchspeed = mavlink_out_griffon_pid_dbg_struct.pid_spd_i;
  mavlink_out_hil_state_struct.rollspeed  = mavlink_out_griffon_pid_dbg_struct.pid_ail_i;
  mavlink_out_hil_state_struct.yawspeed   = mavlink_out_griffon_pid_dbg_struct.pid_ele_i;

  mavlink_out_griffon_surface_angles_struct.r_ail = roll_impact_pid;
  mavlink_out_griffon_surface_angles_struct.l_ail = pitch_impact_pid;
}
#endif
