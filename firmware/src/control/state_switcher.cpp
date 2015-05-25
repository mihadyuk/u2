#include "main.h"
#include "acs.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"
#include "mav_cmd_confirm.hpp"

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
void StateSwitcher::statemode2mavlink(void) {

  mavlink_system_info_struct.state = static_cast<uint8_t>(this->state);
  mavlink_system_info_struct.mode = this->mode;
  if (this->armed)
    mavlink_system_info_struct.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
}

/**
 *
 */
enum MAV_RESULT StateSwitcher::set_mode(const mavlink_set_mode_t *msg) {

  osalSysHalt("TODO: make deep check here");

  if (MAV_STATE_STANDBY == mavlink_system_info_struct.state) {
    mavlink_system_info_struct.mode = clp->param1;
    osalSysHalt("");
    return MAV_RESULT_ACCEPTED;
  }
  else {
    return MAV_RESULT_TEMPORARILY_REJECTED;
  }
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

  case MAV_CMD_DO_SET_MODE:
    result = this->set_mode_command_handler(clp);
    break;

  case MAV_CMD_NAV_TAKEOFF:
    osalSysHalt("unrealized");
    break;

  default:
    result = MAV_RESULT_FAILED;
    break;
  }

  command_ack(result, clp->command, GLOBAL_COMPONENT_ID);
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

    default:
      /* just ignore message */
      break;
    }
  }
}

/**
 *
 */
FutabaResult ACS::analize_futaba(float dT) {
  FutabaResult ret;

  futaba.update(acs_in, dT);

  /* toggle ignore flag for futaba fail */
  if (true == acs_in.futaba_good) {
    if (ManualSwitch::fullauto == acs_in.futaba_man_switch)
      ignore_futaba_fail = true;
    else
      ignore_futaba_fail = false;
  }

  /* futaba fail handling */
  if (!acs_in.futaba_good && !ignore_futaba_fail) {
    ret = FutabaResult::emergency;
  }
  else {
    switch (acs_in.futaba_man_switch) {
    case ManualSwitch::fullauto:
      ret = FutabaResult::fullauto;
      break;
    case ManualSwitch::semiauto:
      ret = FutabaResult::semiauto;
      break;
    case ManualSwitch::manual:
      ret = FutabaResult::manual;
      break;
    default:
      ret = FutabaResult::emergency;
      break;
    }
  }

  return ret;
}

/**
 * @brief   Currently it does nothing.
 */
void ACS::loop_boot(float dT, FutabaResult fr) {
  (void)dT;
  (void)fr;

  state = ACSState::standby;
}

/**
 * @brief   Waits "take_off" command. Perform basic stabilization
 *          for eye checks.
 */
void ACS::loop_standby(float dT, FutabaResult fr) {

  /* */
  switch (fr) {
  case FutabaResult::fullauto:
    stabilizer.update(dT, standby_bytecode);
    mode = MAV_MODE_FLAG_AUTO_ENABLED;
    break;

  case FutabaResult::semiauto:
  case FutabaResult::manual:
    stabilizer.update(dT, manual_bytecode);
    mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    break;

  case FutabaResult::emergency:
    stabilizer.update(dT, emergency_bytecode);
    mode = MAV_MODE_FLAG_AUTO_ENABLED;
    break;
  }
}

/**
 * @brief   Execute mission.
 */
void ACS::loop_active(float dT, FutabaResult fr) {
  MissionState mi_state;
  const uint8_t *auto_bytecode;

  /* */
  switch (fr) {
  case FutabaResult::fullauto:
    mi_state = mission.update(dT);
    auto_bytecode = select_bytecode(mi_state);
    stabilizer.update(dT, auto_bytecode);
    mode = MAV_MODE_FLAG_AUTO_ENABLED;
    break;

  case FutabaResult::semiauto:
    stabilizer.update(dT, semiauto_bytecode);
    mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    break;

  case FutabaResult::manual:
    stabilizer.update(dT, manual_bytecode);
    mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    break;

  case FutabaResult::emergency:
    stabilizer.update(dT, emergency_bytecode);
    mode = MAV_MODE_FLAG_AUTO_ENABLED;
    break;
  }
}

/**
 * @brief   Something goes wrong. Pull hand break or eject 'chute.
 */
void ACS::loop_emergency(float dT, FutabaResult fr) {
  (void)dT;
  (void)fr;

  stabilizer.update(dT, emergency_bytecode);
  mode = MAV_MODE_FLAG_AUTO_ENABLED;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
ACS::ACS(Drivetrain &drivetrain, ACSInput &acs_in) :
drivetrain(drivetrain),
acs_in(acs_in),
stabilizer(impact, acs_in),
command_link(&command_mailbox)
{
  return;
}

/**
 *
 */
void ACS::start(void) {

  futaba.start();
  stabilizer.start();
  drivetrain.start();
  mav_postman.subscribe(MAVLINK_MSG_ID_COMMAND_LONG, &command_link);

  state = ACSState::boot;
}

/**
 *
 */
void ACS::stop(void) {
  state = ACSState::uninit;

  mav_postman.unsubscribe(MAVLINK_MSG_ID_COMMAND_LONG, &command_link);
  command_mailbox.reset();

  drivetrain.stop();
  stabilizer.stop();
  futaba.stop();
}

/**
 *
 */
ACSState ACS::update(float dT) {

  FutabaResult fr;

  osalDbgCheck(ACSState::uninit != state);

  /* first process received messages */
  message_handler();

  /**/
  fr = analize_futaba(dT);

  /**/
  switch (state) {
  case ACSState::boot:
    loop_boot(dT, fr);
    break;
  case ACSState::standby:
    loop_standby(dT, fr);
    break;
  case ACSState::active:
    loop_active(dT, fr);
    break;
  case ACSState::emergency:
    loop_emergency(dT, fr);
    break;
  case ACSState::uninit:
    osalSysHalt(""); /* error trap */
    break;
  }

  drivetrain.update(this->impact);

  statemode2mavlink();
  return this->state;
}

