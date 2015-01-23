#include "main.h"
#include "global_flags.h"
#include "message.hpp"
#include "mav_dispatcher.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define CONFIRM_TMO           MS2ST(500)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_manual_control_t mavlink_in_manual_control_struct;
extern mavlink_set_mode_t mavlink_in_set_mode_struct;
extern mavlink_mission_set_current_t mavlink_in_mission_set_current_struct;

extern EventSource event_mavlink_in_manual_control;
extern EventSource event_mavlink_in_set_mode;
extern EventSource event_mavlink_in_mission_set_current;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/**
 * Command processing thread.
 */
msg_t MavDispatcher::main(void){
  this->setName("MAVCommand");
  eventmask_t evt = 0;

  struct EventListener el_manual_control;
  struct EventListener el_set_mode;
  struct EventListener el_mission_set_current;

  chEvtRegisterMask(&event_mavlink_in_manual_control,       &el_manual_control,       EVMSK_MAVLINK_IN_MANUAL_CONTROL);
  chEvtRegisterMask(&event_mavlink_in_set_mode,             &el_set_mode,             EVMSK_MAVLINK_IN_SET_MODE);
  chEvtRegisterMask(&event_mavlink_in_mission_set_current,  &el_mission_set_current,  EVMSK_MAVLINK_IN_MISSION_SET_CURRENT);

  /* wait modems */
  while(GlobalFlags.messaging_ready == 0)
    chThdSleepMilliseconds(50);

  /* main loop */
  while(!chThdShouldTerminate()){
    evt = chEvtWaitOneTimeout(EVMSK_MAVLINK_IN_MANUAL_CONTROL |
                              EVMSK_MAVLINK_IN_SET_MODE |
                              EVMSK_MAVLINK_IN_MISSION_SET_CURRENT,
                              MS2ST(50));

    switch (evt){
    case EVMSK_MAVLINK_IN_MANUAL_CONTROL:
      acs.manualControl(&mavlink_in_manual_control_struct);
      break;

    case EVMSK_MAVLINK_IN_SET_MODE:
      acs.requestSetMode(&mavlink_in_set_mode_struct);
      break;

    case EVMSK_MAVLINK_IN_MISSION_SET_CURRENT:
      acs.setCurrentMission(&mavlink_in_mission_set_current_struct);
      break;

    default:
      break;
    }
  }

  chEvtUnregister(&event_mavlink_in_manual_control,       &el_manual_control);
  chEvtUnregister(&event_mavlink_in_set_mode,             &el_set_mode);
  chEvtUnregister(&event_mavlink_in_mission_set_current,  &el_mission_set_current);
  chThdExit(0);
  return 0;
}


/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
MavDispatcher::MavDispatcher(ACS &acsp) :
acs(acsp)
{
  return;
}
