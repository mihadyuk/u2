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
enum MAV_RESULT ACS::alcoi_command_handler(const mavlink_command_long_t *clp) {
  AlcoiPulse pulse;

  pulse.pid      = roundf(clp->PARAM_PULSE_CHANNEL);
  pulse.width   = clp->PARAM_PULSE_WIDTH;
  pulse.strength= clp->PARAM_PULSE_STRENGTH;

  if (pulse.width > ALCOI_MAX_PULSE_WIDTH)
    return MAV_RESULT_FAILED;

  if (OSAL_SUCCESS == stabilizer.alcoiPulse(pulse))
    return MAV_RESULT_ACCEPTED;
  else
    return MAV_RESULT_FAILED;
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
void ACS::alcoi_handler(void) {

  mavMail *recv_mail;

  if (MSG_OK == command_mailbox.fetch(&recv_mail, TIME_IMMEDIATE)) {
    if (MAVLINK_MSG_ID_COMMAND_LONG == recv_mail->msgid) {
      command_long_handler(recv_mail);
    }
  }
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
    alcoi_handler();
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

