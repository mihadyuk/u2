#include "main.h"
#include "acs.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"
#include "mav_dbg.hpp"
#include "mav_cmd_confirm.hpp"

using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
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
 * @brief   This function handles only MAV_CMD_DO_SET_SERVO
 */
static void command_long_handler(const mavMail *recv_mail, Alcoi &alcoi) {

  enum MAV_RESULT result = MAV_RESULT_FAILED;
  bool status = OSAL_FAILED;
  const mavlink_command_long_t *clp
  = static_cast<const mavlink_command_long_t *>(recv_mail->mavmsg);

  if (!mavlink_msg_for_me(clp))
    return;

  if (MAV_CMD_DO_SET_SERVO != clp->command)
    return;

  AlcoiPulse pulse;
  pulse.lvl     = static_cast<OverrideLevel>(roundf(clp->PARAM_PULSE_LEVEL));
  pulse.ch      = static_cast<pid_chain_t>(roundf(clp->PARAM_PULSE_CHANNEL));
  pulse.width   = clp->PARAM_PULSE_WIDTH;
  pulse.strength= clp->PARAM_PULSE_STRENGTH;

  status = alcoi.loadPulse(pulse);
  if (OSAL_SUCCESS == status) {
    mavlink_dbg_print(MAV_SEVERITY_INFO, "OK: Alcoi pulse accepted", GLOBAL_COMPONENT_ID);
    result = MAV_RESULT_ACCEPTED;
  }
  else {
    result = MAV_RESULT_FAILED;
  }

  command_ack(result, clp->command, GLOBAL_COMPONENT_ID);
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
      command_long_handler(recv_mail, this->alcoi);
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

