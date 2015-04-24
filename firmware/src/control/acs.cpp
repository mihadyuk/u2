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
    INPUT,  ACS_INPUT_wx,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  ACS_INPUT_wy,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

static const uint8_t semiauto_program[] = {
    INPUT,  ACS_INPUT_roll,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  ACS_INPUT_pitch,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

static const uint8_t manual_program[] = {
    INPUT,  ACS_INPUT_futaba_raw_00,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  ACS_INPUT_futaba_raw_01,
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
ACS::ACS(Drivetrain &drivetrain, ACSInput &acs_in) :
drivetrain(drivetrain),
acs_in(acs_in),
stabilizer(impact, acs_in),
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
void futaba2state_vector(const FutabaOutput &fut_data, ACSInput &sv) {
  sv.ch[ACS_INPUT_futaba_raw_00] = fut_data.ch[0];
  sv.ch[ACS_INPUT_futaba_raw_01] = fut_data.ch[1];
  sv.ch[ACS_INPUT_futaba_raw_02] = fut_data.ch[2];
  sv.ch[ACS_INPUT_futaba_raw_03] = fut_data.ch[3];
}

/**
 *
 */
void ACS::update(float dT) {

  osalDbgCheck(ready);

  futaba.update(acs_in, dT);

  /* toggle ignore flag for futaba fail */
  if (true == acs_in.futaba_good) {
    if (ManualSwitch::fullauto == acs_in.futaba_man)
      ignore_futaba_fail = true;
    else
      ignore_futaba_fail = false;
  }

  /* futaba fail handling */
  if (!ignore_futaba_fail && (false == acs_in.futaba_good))
    return this->failsafe();

  /* main code */
  switch (acs_in.futaba_man) {
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

