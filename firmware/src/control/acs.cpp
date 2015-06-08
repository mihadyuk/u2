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

static const uint8_t auto_bytecode[] = {
    INPUT,  ACS_INPUT_pitch,
    OUTPUT, IMPACT_RUD,
    TERM,

//    INPUT, ACS_INPUT_odo_speed,
//    PID, 15, ACS_INPUT_const_one,
//    OUTPUT, IMPACT_THR,
//    TERM,

    END
};

static const uint8_t semiauto_bytecode[] = {
    INPUT,  ACS_INPUT_roll,
    OUTPUT, IMPACT_RUD,
    SCALE, 0,
    TERM,

    INPUT,  ACS_INPUT_wy,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

static const uint8_t manual_bytecode[] = {
    INPUT,  ACS_INPUT_futaba_raw_00,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  ACS_INPUT_futaba_raw_01,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

static const uint8_t emergency_bytecode[] = {
    INPUT,  ACS_INPUT_const_zero,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  ACS_INPUT_const_zero,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

static const uint8_t standby_bytecode[] = {
    INPUT,  ACS_INPUT_roll,
    SCALE,  0,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  ACS_INPUT_futaba_raw_01,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

static const uint8_t loiter_bytecode[] = {
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
 * @brief   Currently it does nothing.
 */
void ACS::loop_takeoff(float dT, FutabaResult fr) {
  (void)dT;
  (void)fr;

  if (OSAL_SUCCESS == mission.takeoff()) {
    state = ACSState::navigate;
  }
}

/**
 *
 */
void ACS::reached_handler(void) {
  bool load_status = OSAL_FAILED;

  switch (mission.getTrgtCmd()) {
  case MAV_CMD_NAV_WAYPOINT:
    load_status = mission.loadNext();
    if (OSAL_FAILED == load_status) {
      osalSysHalt("");
      this->state = ACSState::emergency;
    }
    break;

  /**/
  case MAV_CMD_NAV_LOITER_UNLIM:
  case MAV_CMD_NAV_LOITER_TIME:
  case MAV_CMD_NAV_LOITER_TURNS:
    this->state = ACSState::loiter;
    break;

  /**/
  default:
    load_status = mission.loadNext();
    if (OSAL_FAILED == load_status) {
      osalSysHalt("");
      this->state = ACSState::emergency;
    }
    break;
  }
}

/**
 * @brief   Execute mission.
 */
void ACS::loop_navigate(float dT) {

  MissionState mi_status = mission.update();

  stabilizer.update(dT, auto_bytecode);

  switch (mi_status) {
  case MissionState::reached:
    reached_handler();
    break;

  /**/
  case MissionState::completed:
    this->state = ACSState::loiter;
    break;

  /**/
  case MissionState::error:
    osalSysHalt("");
    this->state = ACSState::emergency;
    break;

  /**/
  case MissionState::navigate:
    break;

  /**/
  default:
    osalSysHalt("Unhandled case");
    break;
  }
}

/**
 * @brief   Something goes wrong. Pull hand break or eject 'chute.
 */
void ACS::loop_manual(float dT, FutabaResult fr) {
  (void)dT;
  (void)fr;

  osalSysHalt("unrealized");
  stabilizer.update(dT, emergency_bytecode);
  mode = MAV_MODE_FLAG_AUTO_ENABLED;
}

/**
 * @brief   Something goes wrong. Pull hand break or eject 'chute.
 */
void ACS::loop_semiauto(float dT, FutabaResult fr) {
  (void)dT;
  (void)fr;

  osalSysHalt("unrealized");
  stabilizer.update(dT, emergency_bytecode);
  mode = MAV_MODE_FLAG_AUTO_ENABLED;
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
void ACS::loop_loiter(float dT, FutabaResult fr) {
  (void)dT;
  (void)fr;

  stabilizer.update(dT, loiter_bytecode);
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

  futaba.start();
  stabilizer.start();
  drivetrain.start();
  mav_postman.subscribe(MAVLINK_MSG_ID_COMMAND_LONG, &command_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_SET_MODE, &set_mode_link);

  state = ACSState::standby;
}

/**
 *
 */
void ACS::stop(void) {
  state = ACSState::uninit;

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
ACSState ACS::update(float dT) {

  FutabaResult fr;

  /* first process received messages */
  message_handler();

  /**/
  fr = analize_futaba(dT);

  /**/
  switch (state) {
  case ACSState::standby:
    loop_standby(dT, fr);
    break;
  case ACSState::navigate:
    loop_navigate(dT);
    break;
  case ACSState::loiter:
    loop_loiter(dT, fr);
    break;
  case ACSState::emergency:
    loop_emergency(dT, fr);
    break;
  default:
    osalSysHalt("Unhandled case"); /* error trap */
    break;
  }

  drivetrain.update(this->impact);

  return this->state;
}

