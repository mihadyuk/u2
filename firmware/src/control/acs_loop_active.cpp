#include "main.h"

#include "acs.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"
#include "mav_postman.hpp"
#include "pads.h"

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

static const uint8_t auto_bytecode[] = {
    INPUT,  ACS_INPUT_pitch,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT, ACS_INPUT_const_two,
    PID, 15, ACS_INPUT_odo_speed,
    OUTPUT, IMPACT_THR,
    TERM,

    END
};

static const uint8_t semiauto_bytecode[] = {
    INPUT,  ACS_INPUT_roll,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  ACS_INPUT_futaba_raw_01,
    SCALE, 0,
    PID, 15, ACS_INPUT_odo_speed,
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

static const uint8_t loiter_bytecode[] = {
    INPUT,  ACS_INPUT_const_zero,
    OUTPUT, IMPACT_RUD,
    TERM,

    INPUT,  ACS_INPUT_const_zero,
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
void ACS::reached_handler(void) {
  bool load_status = OSAL_FAILED;

  switch (mission.getTrgtCmd()) {
  case MAV_CMD_NAV_WAYPOINT:
    load_status = mission.loadNext();
    if (OSAL_FAILED == load_status) {
      this->nav_substate = NavigateSubstate::loiter;
    }
    break;

  /**/
  case MAV_CMD_NAV_LOITER_UNLIM:
  case MAV_CMD_NAV_LOITER_TIME:
  case MAV_CMD_NAV_LOITER_TURNS:
    this->nav_substate = NavigateSubstate::loiter;
    break;

  /**/
  default:
    load_status = mission.loadNext();
    if (OSAL_FAILED == load_status) {
      this->nav_substate = NavigateSubstate::loiter;
    }
    break;
  }
}

/**
 *
 */
void ACS::loop_active_navigate_mission(float dT) {

  MissionState mi_status = mission.update();

  switch (mi_status) {
  case MissionState::reached:
    reached_handler();
    break;

  /**/
  case MissionState::completed:
    this->nav_substate = NavigateSubstate::loiter;
    break;

  /**/
  case MissionState::error:
    this->nav_substate = NavigateSubstate::loiter;
    mavlink_system_info_struct.state = MAV_STATE_EMERGENCY;
    break;

  /**/
  case MissionState::navigate:
    stabilizer.update(dT, auto_bytecode);
    blue_led_toggle();
    break;

  /**/
  default:
    osalSysHalt("Unhandled case");
    break;
  }

  mode = MAV_MODE_FLAG_AUTO_ENABLED;
}

/**
 * @brief   Something goes wrong. Pull hand break or eject 'chute.
 */
void ACS::loop_active_navigate(float dT) {

  switch (nav_substate) {
  case NavigateSubstate::mission:
    loop_active_navigate_mission(dT);
    break;
  case NavigateSubstate::land:
    loop_active_navigate_land(dT);
    break;
  case NavigateSubstate::loiter:
    loop_active_navigate_loiter(dT);
    break;
  default:
    osalSysHalt("Unhandled case");
    break;
  }

  mode = MAV_MODE_FLAG_AUTO_ENABLED;
}

/**
 * @brief   Execute mission.
 */
void ACS::loop_active(float dT, FutabaResult fr) {

  switch (fr) {
  case FutabaResult::fullauto:
    active_substate = ActiveSubstate::navigate;
    break;
  case FutabaResult::semiauto:
    active_substate = ActiveSubstate::semiauto;
    break;
  case FutabaResult::manual:
    active_substate = ActiveSubstate::manual;
    break;
  case FutabaResult::emergency:
    mavlink_system_info_struct.state = MAV_STATE_EMERGENCY;
    break;
  }
  if (mavlink_system_info_struct.state == MAV_STATE_EMERGENCY)
    return;

  /* */
  switch (active_substate) {
  case ActiveSubstate::navigate:
    loop_active_navigate(dT);
    break;
  case ActiveSubstate::semiauto:
    loop_active_semiauto(dT);
    break;
  case ActiveSubstate::manual:
    loop_active_manual(dT);
    break;
  default:
    osalSysHalt("Unhandled case");
    break;
  }
}

/**
 *
 */
void ACS::loop_active_manual(float dT) {

  stabilizer.update(dT, manual_bytecode);
  mode = MAV_MODE_FLAG_AUTO_ENABLED;
}

/**
 *
 */
void ACS::loop_active_semiauto(float dT) {

  stabilizer.update(dT, semiauto_bytecode);
  mode = MAV_MODE_FLAG_AUTO_ENABLED;
}

/**
 *
 */
void ACS::loop_active_navigate_loiter(float dT) {

  stabilizer.update(dT, loiter_bytecode);
  mode = MAV_MODE_FLAG_AUTO_ENABLED;
}

/**
 *
 */
void ACS::loop_active_navigate_land(float dT) {
  (void)dT;

  blue_led_off();
  nav_substate = NavigateSubstate::mission;
  mavlink_system_info_struct.state = MAV_STATE_STANDBY;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

