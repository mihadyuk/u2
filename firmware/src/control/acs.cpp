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
void ACS::failsafe(void) {
  return;
}

static void navigator(StabInput &result) {

  for (size_t i=0; i<PID_CHAIN_ENUM_END; i++) {
    result.ch[i] = 0;
    result.ol[i] = OverrideLevel::high;
  }
}

/**
 *
 */
static void futaba2stab_input(const FutabaOutput &fut, StabInput &result) {

  memcpy(result.ch, fut.ch, sizeof(fut.ch));
  memcpy(result.ol, fut.ol, sizeof(fut.ol));
}

/**
 * @brief   This function handles only MAV_CMD_DO_SET_SERVO
 */
static void command_long_handler(const mavMail *recv_mail) {

  enum MAV_RESULT result = MAV_RESULT_FAILED;
  bool status = OSAL_FAILED;
  const mavlink_command_long_t *clp
  = static_cast<const mavlink_command_long_t *>(recv_mail->mavmsg);

  if (!mavlink_msg_for_me(clp))
    return;

  if (MAV_CMD_DO_SET_SERVO != clp->command)
    return;

  /* Mavlink protocol claims "This command will be only accepted if
     in pre-flight mode." But in our realization allows to use it in any time. */
  //if ((mavlink_system_struct.mode != MAV_MODE_PREFLIGHT) || (mavlink_system_struct.state != MAV_STATE_STANDBY)){
  //   result = MAV_RESULT_TEMPORARILY_REJECTED;
  //}
  //else{
  mavlink_dbg_print(MAV_SEVERITY_INFO, "eeprom operation started", GLOBAL_COMPONENT_ID);
  if (roundf(clp->param1) == 0)
    status = param_registry.loadToRam();
  else if (roundf(clp->param1) == 1)
    status = param_registry.saveAll();

  if (status != OSAL_SUCCESS){
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "ERROR: eeprom operation failed", GLOBAL_COMPONENT_ID);
    result = MAV_RESULT_FAILED;
  }
  else{
    mavlink_dbg_print(MAV_SEVERITY_INFO, "OK: eeprom operation success", GLOBAL_COMPONENT_ID);
    result = MAV_RESULT_ACCEPTED;
  }

  command_ack(result, clp->command, GLOBAL_COMPONENT_ID);
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
    semiauto(dT, fut_data);
    break;
  case ManualSwitch::manual:
    manual(dT, fut_data);
    break;
  }
}

