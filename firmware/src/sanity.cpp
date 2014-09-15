#include "main.h"
#include "fault_handlers.h"
#include "pads.h"

#include "mavworker.hpp"
#include "message.hpp"
#include "bkp.hpp"
#include "mavlogger.hpp"
#include "blinker.hpp"
#include "cpu_load.h"
#include "sanity.hpp"

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern mavlink_system_t       mavlink_system_struct;
extern mavlink_heartbeat_t    mavlink_out_heartbeat_struct;
extern mavlink_sys_status_t   mavlink_out_sys_status_struct;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define HEART_BEAT_PERIOD   MS2ST(1000)

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
static const int16_t normal_seq[]      = {50,  -100, 0};
static const int16_t panic_seq[]       = {50,  -100, 50, -100, 50, -100, 50, -100, 0};
static const int16_t calibrating_seq[] = {800, -200, 0};

static mavMail hearbeat_mail;

static uint32_t LastResetFlags;

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/**
 * Heartbeat blinker logic
 */
static void heartbeat_blinker(void){
  if (was_softreset()){
    blinker.normal_post(&panic_seq[0]);
  }
  else{
    if (MAV_STATE_CALIBRATING == mavlink_system_struct.state)
      blinker.normal_post(calibrating_seq);
    else
      blinker.normal_post(normal_seq);
  }
}

/**
 *
 */
static THD_WORKING_AREA(SanityControlThreadWA, 144);
static THD_FUNCTION(SanityControlThread, arg) {
  chRegSetThreadName("Sanity");
  (void)arg;

  mavlink_out_heartbeat_struct.autopilot = MAV_AUTOPILOT_GENERIC;
  mavlink_out_heartbeat_struct.custom_mode = 0;

  systime_t t = chVTGetSystemTimeX();

  while (!chThdShouldTerminateX()) {
    t += HEART_BEAT_PERIOD;

    /* fill data fields */
    mavlink_out_heartbeat_struct.type           = mavlink_system_struct.type;
    mavlink_out_heartbeat_struct.base_mode      = mavlink_system_struct.mode;
    mavlink_out_heartbeat_struct.system_status  = mavlink_system_struct.state;

    /* schedule sending over telemetry channel */
    hearbeat_mail.fill(&mavlink_out_heartbeat_struct, MAV_COMP_ID_SYSTEM_CONTROL, MAVLINK_MSG_ID_HEARTBEAT);
    mav_worker.post(hearbeat_mail);

    mavlink_out_sys_status_struct.load = getCpuLoad();
    /* how many times device was soft reseted */
    mavlink_out_sys_status_struct.errors_count1 = bkpSoftResetCnt;
    /* reset flags */
    mavlink_out_sys_status_struct.errors_count2 = LastResetFlags >> 24;

    heartbeat_blinker();
    chThdSleepUntil(t);
  }

  chThdExit(MSG_OK);
  return MSG_OK;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
void SanityControlInit(void){

  /* write soft reset event to "log" and clean it in case of pad reset */
  if (was_softreset())
    bkpSoftResetCnt++;

  LastResetFlags = RCC->CSR;
  clear_reset_flags();

  /* hardcoded enumeration of presented sensors */
  mavlink_out_sys_status_struct.onboard_control_sensors_present = (
              SANITY_3D_GYRO | SANITY_3D_ACCEL | SANITY_3D_MAG |
              SANITY_ABS_PRES | SANITY_DIFF_PRES | SANITY_GPS |
              SANITY_3D_ANGULAR_RATE_CONTROL | SANITY_ATTITUDE_STABILIZATION |
              SANITY_Z_CONTROL | SANITY_XY_CONTROL | SANITY_MOTOR_CONTROL |
              SANITY_RC_RECEIVER);
  mavlink_out_sys_status_struct.onboard_control_sensors_enabled = mavlink_out_sys_status_struct.onboard_control_sensors_present;
  mavlink_out_sys_status_struct.onboard_control_sensors_health = mavlink_out_sys_status_struct.onboard_control_sensors_present;

  /* */
  chThdCreateStatic(SanityControlThreadWA,
          sizeof(SanityControlThreadWA),
          NORMALPRIO,
          SanityControlThread,
          NULL);

}



