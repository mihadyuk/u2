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
#define PARAM_PULSE_LEVEL         param4
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
      this->state = ACSState::navigate;
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
void ACS::command_long_handler(const mavMail *recv_mail) {

  enum MAV_RESULT result = MAV_RESULT_FAILED;
  const mavlink_command_long_t *clp =
      static_cast<const mavlink_command_long_t *>(recv_mail->mavmsg);

  if (!mavlink_msg_for_me(clp))
    return;

  switch (clp->command) {
  case MAV_CMD_DO_SET_SERVO:
    result = this->alcoi_command_handler(clp);
    break;

  case MAV_CMD_NAV_TAKEOFF:
    result = this->take_off_handler(clp);
    break;

  case MAV_CMD_PREFLIGHT_CALIBRATION:
    result = this->calibrate_command_handler(clp);
    break;

  default:
    goto SILENT_EXIT;
    break;
  }
  command_ack(result, clp->command, GLOBAL_COMPONENT_ID);

SILENT_EXIT:
  return;
}

/**
 *
 */
void ACS::set_mode_handler(const mavMail *recv_mail) {

  const mavlink_set_mode_t *smp =
      static_cast<const mavlink_set_mode_t *>(recv_mail->mavmsg);

  if (smp->target_system != mavlink_system_struct.sysid)
    return; /* it is not for our system */

  /* mode may changed only in standby state */
  if (MAV_STATE_STANDBY == mavlink_system_info_struct.state) {
    mavlink_system_info_struct.mode = smp->base_mode;

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

  mavMail *recv_mail;

  if (MSG_OK == command_mailbox.fetch(&recv_mail, TIME_IMMEDIATE)) {
    switch (recv_mail->msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG:
      command_long_handler(recv_mail);
      break;

    case MAVLINK_MSG_ID_SET_MODE:
      set_mode_handler(recv_mail);
      break;

    default:
      /* just ignore unhandled message */
      break;
    }
  }
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

