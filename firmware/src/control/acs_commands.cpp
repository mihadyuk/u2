#include "main.h"
#include "acs.hpp"

#include "mavlink_local.hpp"
#include "mav_cmd_confirm.hpp"
#include "mav_dbg.hpp"

using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define ALCOI_MAX_PULSE_WIDTH     2 // seconds

/* convenience defines */
#define PARAM_PULSE_CHANNEL       param5
#define PARAM_PULSE_WIDTH         param6
#define PARAM_PULSE_STRENGTH      param7

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern mavlink_system_info_t   mavlink_system_info_struct;
extern mavlink_system_t        mavlink_system_struct;

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

/**
 *
 */
enum MAV_RESULT ACS::alcoi_command_handler(const mavlink_command_long_t *clp) {

  if (ManualSwitch::fullauto != acs_in.futaba_man_switch) {
    return MAV_RESULT_TEMPORARILY_REJECTED;
  }
  else {
    AlcoiPulse pulse;

    pulse.pid     = roundf(clp->PARAM_PULSE_CHANNEL);
    pulse.width   = clp->PARAM_PULSE_WIDTH;
    pulse.strength= clp->PARAM_PULSE_STRENGTH;

    if (pulse.width > ALCOI_MAX_PULSE_WIDTH)
      return MAV_RESULT_FAILED;

    if (OSAL_SUCCESS == stabilizer.alcoiPulse(pulse))
      return MAV_RESULT_ACCEPTED;
    else
      return MAV_RESULT_FAILED;
  }
}

/**
 *
 */
enum MAV_RESULT ACS::calibrate_command_handler(const mavlink_command_long_t *clp) {
  (void)clp;

  /* mode may changed only in standby state */
  if (MAV_STATE_STANDBY == mavlink_system_info_struct.state) {
    mavlink_system_info_struct.state = MAV_STATE_CALIBRATING;
    return MAV_RESULT_ACCEPTED;
  }
  else {
    return MAV_RESULT_TEMPORARILY_REJECTED;
  }
}

/**
 *
 */
enum MAV_RESULT ACS::take_off_handler(const mavlink_command_long_t *clp) {
  (void)clp;
  enum MAV_RESULT result = MAV_RESULT_FAILED;

  /* mode may changed only in standby state */
  if (MAV_STATE_STANDBY == mavlink_system_info_struct.state) {
    if (OSAL_SUCCESS == mission.takeoff()) {
      mavlink_system_info_struct.state = MAV_STATE_ACTIVE;
      result = MAV_RESULT_ACCEPTED;
    }
    else {
      result = MAV_RESULT_FAILED;
    }
  }
  else {
    result = MAV_RESULT_TEMPORARILY_REJECTED;
  }

  return result;
}

/**
 *
 */
enum MAV_RESULT ACS::land_handler(const mavlink_command_long_t *clp) {
  (void)clp;
  enum MAV_RESULT result = MAV_RESULT_FAILED;

  nav_substate = NavigateSubstate::land;
  result = MAV_RESULT_ACCEPTED;

  return result;
}

/**
 *
 */
void ACS::command_long_handler(const mavlink_message_t *recv_msg) {

  enum MAV_RESULT result = MAV_RESULT_FAILED;
  mavlink_command_long_t cl;

  mavlink_msg_command_long_decode(recv_msg, &cl);

  if (! mavlink_msg_for_me(&cl))
    goto SILENT_EXIT;

  switch (cl.command) {
  case MAV_CMD_DO_SET_SERVO:
    result = this->alcoi_command_handler(&cl);
    break;

  case MAV_CMD_NAV_TAKEOFF:
    result = this->take_off_handler(&cl);
    break;

  case MAV_CMD_NAV_LAND:
    result = this->land_handler(&cl);
    break;

  case MAV_CMD_PREFLIGHT_CALIBRATION:
    result = this->calibrate_command_handler(&cl);
    break;

  default:
    goto SILENT_EXIT;
    break;
  }

  command_ack(result, cl.command, GLOBAL_COMPONENT_ID);

SILENT_EXIT:
  return;
}

/**
 *
 */
void ACS::set_mode_handler(const mavlink_message_t *recv_msg) {

  mavlink_set_mode_t sm;

  mavlink_msg_set_mode_decode(recv_msg, &sm);

  if (sm.target_system != mavlink_system_struct.sysid)
    return; /* it is not for our system */

  /* mode may changed only in standby state */
  if (MAV_STATE_STANDBY == mavlink_system_info_struct.state) {
    mavlink_system_info_struct.mode = sm.base_mode;

    /* process "armed" flag */
    if (mavlink_system_info_struct.mode | MAV_MODE_FLAG_SAFETY_ARMED)
      this->drivetrain.arm();
    else
      this->drivetrain.disarm();
  }
  else {
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "Can not change mode when system active", MAV_COMP_ID_ALL);
  }
}

/**
 *
 */
void ACS::message_handler(void) {

  mavlink_message_t *recv_msg;

  if (MSG_OK == command_mailbox.fetch(&recv_msg, TIME_IMMEDIATE)) {
    switch (recv_msg->msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG:
      command_long_handler(recv_msg);
      break;

    case MAVLINK_MSG_ID_SET_MODE:
      set_mode_handler(recv_msg);
      break;

    default:
      /* just ignore unhandled message */
      break;
    }

    mav_postman.free(recv_msg);
  }
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

