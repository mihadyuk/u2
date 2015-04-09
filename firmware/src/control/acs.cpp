#include "main.h"
#include "acs.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"
#include "mav_cmd_confirm.hpp"
//#include "mav_dbg.hpp"

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
void ACS::command_long_handler(const mavMail *recv_mail) {

  enum MAV_RESULT result = MAV_RESULT_FAILED;
  const mavlink_command_long_t *clp =
      static_cast<const mavlink_command_long_t *>(recv_mail->mavmsg);

  if (!mavlink_msg_for_me(clp))
    return;

  switch (clp->command) {
  case MAV_CMD_DO_SET_SERVO:
    result = this->alcoi.commandHandler(clp);
    break;
  default:
    break;
  }

  command_ack(result, clp->command, GLOBAL_COMPONENT_ID);
}

/**
 * TODO:
 */
void ACS::failsafe(void) {

  return;
}

/**
 * TODO:
 */
static void navigator(StabInput &result) {

  for (size_t i=0; i<PID_CHAIN_ENUM_END; i++) {
    result.ch[i].target = 0;
    result.ch[i].override_target = 0;
    result.ch[i].override_level = OverrideLevel::high;
  }
}

/**
 *  TODO:
 */
static void futaba2stab_input(const FutabaOutput &fut, StabInput &result) {

  for (size_t i=0; i<PID_CHAIN_ENUM_END; i++) {
    result.ch[i].override_target = fut.ch[i];
    result.ch[i].override_level  = fut.ol[i];
  }
}

/**
 *
 */
void ACS::fullauto(float dT, const FutabaOutput &fut_data) {
  (void)fut_data;
  StabInput stab_input;
  mavMail *recv_mail;

  if (MSG_OK == command_mailbox.fetch(&recv_mail, TIME_IMMEDIATE)) {
    if (MAVLINK_MSG_ID_COMMAND_LONG == recv_mail->msgid) {
      command_long_handler(recv_mail);
    }
  }

  navigator(stab_input);
  alcoi.update(stab_input, dT);
  stabilizer.update(stab_input, dT);
}

/**
 *
 */
void ACS::semiauto(float dT, const FutabaOutput &fut_data) {
  (void)fut_data;
  StabInput stab_input;

  navigator(stab_input);
  stabilizer.update(stab_input, dT);
}

/**
 *
 */
void ACS::manual(float dT, const FutabaOutput &fut_data) {
  StabInput stab_input;

  futaba2stab_input(fut_data, stab_input);
  stabilizer.update(stab_input, dT);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
ACS::ACS(Drivetrain &drivetrain, const StateVector &s) :
stabilizer(drivetrain, s),
command_long_link(&command_mailbox)
{
  return;
}

/**
 *
 */
void ACS::start(void) {

  stabilizer.start();
  futaba.start();
  alcoi.start();
  mav_postman.subscribe(MAVLINK_MSG_ID_COMMAND_LONG, &command_long_link);

  ready = true;
}

/**
 *
 */
void ACS::stop(void) {
  ready = false;

  mav_postman.unsubscribe(MAVLINK_MSG_ID_COMMAND_LONG, &command_long_link);
  command_mailbox.reset();

  futaba.stop();
  stabilizer.stop();
}

/**
 *
 */
void ACS::update(float dT) {
  FutabaOutput fut_data;
  msg_t futaba_status = MSG_OK;

  osalDbgCheck(ready);

  futaba_status = futaba.update(fut_data, dT);

  /* toggle ignore flag for futaba fail */
  if (MSG_OK == futaba_status) {
    if (ManualSwitch::fullauto == fut_data.man)
      ignore_futaba_fail = true;
    else
      ignore_futaba_fail = false;
  }

  /* futaba fail handling */
  if (!ignore_futaba_fail && (MSG_OK != futaba_status))
    return this->failsafe();

  /* main code */
  switch (fut_data.man) {
  case ManualSwitch::fullauto:
    fullauto(dT, fut_data);
    break;
  case ManualSwitch::semiauto:
    command_mailbox.reset(); // prevent spurious pulse execution when switch to full auto mode
    semiauto(dT, fut_data);
    break;
  case ManualSwitch::manual:
    command_mailbox.reset(); // prevent spurious pulse execution when switch to full auto mode
    manual(dT, fut_data);
    break;
  }
}

