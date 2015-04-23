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

static const uint8_t auto_program[] = {
    INPUT,  STATE_VECTOR_wx,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  STATE_VECTOR_wy,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

static const uint8_t semiauto_program[] = {
    INPUT,  STATE_VECTOR_roll,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  STATE_VECTOR_pitch,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

static const uint8_t manual_program[] = {
    INPUT,  STATE_VECTOR_futaba_raw_00,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  STATE_VECTOR_futaba_raw_01,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

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
 *
 */
void ACS::fullauto(float dT, const FutabaOutput &fut_data) {
  (void)fut_data;
  mavMail *recv_mail;

  if (MSG_OK == command_mailbox.fetch(&recv_mail, TIME_IMMEDIATE)) {
    if (MAVLINK_MSG_ID_COMMAND_LONG == recv_mail->msgid) {
      command_long_handler(recv_mail);
    }
  }

  alcoi.update(dT);
}

/**
 *
 */
void ACS::semiauto(float dT, const FutabaOutput &fut_data) {
  (void)fut_data;
  (void)dT;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
ACS::ACS(Drivetrain &drivetrain, StateVector &sv) :
drivetrain(drivetrain),
sv(sv),
stabilizer(impact, sv),
command_long_link(&command_mailbox)
{
  return;
}

/**
 *
 */
void ACS::start(void) {

  futaba.start();
  alcoi.start();
  stabilizer.start();
  drivetrain.start();
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

  drivetrain.stop();
  stabilizer.stop();
  futaba.stop();
}

/**
 *
 */
void futaba2state_vector(const FutabaOutput &fut_data, StateVector &sv) {
  sv.ch[STATE_VECTOR_futaba_raw_00] = fut_data.ch[0];
  sv.ch[STATE_VECTOR_futaba_raw_01] = fut_data.ch[1];
  sv.ch[STATE_VECTOR_futaba_raw_02] = fut_data.ch[2];
  sv.ch[STATE_VECTOR_futaba_raw_03] = fut_data.ch[3];
}

/**
 *
 */
void ACS::update(float dT) {
  FutabaOutput fut_data;
  msg_t futaba_status = MSG_OK;

  osalDbgCheck(ready);

  futaba_status = futaba.update(fut_data, dT);
  if (MSG_OK == futaba_status)
    futaba2state_vector(fut_data, this->sv);

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
    stabilizer.update(dT, auto_program);
    break;

  case ManualSwitch::semiauto:
    command_mailbox.reset(); // prevent spurious pulse execution when switch to full auto mode
    stabilizer.update(dT, semiauto_program);
    break;

  case ManualSwitch::manual:
    command_mailbox.reset(); // prevent spurious pulse execution when switch to full auto mode
    stabilizer.update(dT, manual_program);
    break;
  }

  drivetrain.update(this->impact);
}

