#include "main.h"

#include "acs.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"
#include "mav_postman.hpp"

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
extern mavlink_system_info_t   mavlink_system_info_struct;

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

static const uint8_t emergency_bytecode[] = {
    INPUT,  ACS_INPUT_const_zero,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  ACS_INPUT_const_zero,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

static const uint8_t critical_bytecode[] = {
    INPUT,  ACS_INPUT_const_zero,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  ACS_INPUT_const_zero,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

static const uint8_t standby_bytecode[] = {
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
    stabilizer.update(dT, standby_bytecode);
    mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    break;

  case FutabaResult::emergency:
    stabilizer.update(dT, standby_bytecode);
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

/**
 * @brief   Something goes wrong. Pull hand break or eject 'chute.
 */
void ACS::loop_critical(float dT, FutabaResult fr) {
  (void)dT;
  (void)fr;

  stabilizer.update(dT, critical_bytecode);
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
mission(acs_in),
stabilizer(impact, acs_in),
command_link(&command_mailbox),
set_mode_link(&command_mailbox)
{
  return;
}

/**
 *
 */
void ACS::start(void) {

  param_registry.valueSearch("SPD_trgt_speed",&trgt_speed);
  param_registry.valueSearch("SPD_speed_max", &speed_max);

  futaba.start();
  stabilizer.start();
  drivetrain.start();
  mav_postman.subscribe(MAVLINK_MSG_ID_COMMAND_LONG, &command_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_SET_MODE, &set_mode_link);

  ready = true;
}

/**
 *
 */
void ACS::stop(void) {
  ready = false;

  mav_postman.unsubscribe(MAVLINK_MSG_ID_SET_MODE, &set_mode_link);
  mav_postman.unsubscribe(MAVLINK_MSG_ID_COMMAND_LONG, &command_link);
  command_mailbox.reset();

  drivetrain.stop();
  stabilizer.stop();
  futaba.stop();
}

/**
 *
 */
void ACS::update(float dT) {

  acs_in.ch[ACS_INPUT_trgt_speed] = *this->trgt_speed;

  osalDbgCheck(ready);

  FutabaResult fr;

  /* first process received messages */
  message_handler();

  /**/
  fr = analize_futaba(dT);

  switch (mavlink_system_info_struct.state) {
  case MAV_STATE_STANDBY:
    loop_standby(dT, fr);
    break;
  case MAV_STATE_ACTIVE:
    loop_active(dT, fr);
    break;
  case MAV_STATE_CRITICAL:
    loop_critical(dT, fr);
    break;
  case MAV_STATE_EMERGENCY:
    loop_emergency(dT, fr);
    break;
  default:
    break;
  }

  drivetrain.update(this->impact);
}

