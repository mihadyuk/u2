#include "main.h"
#include "message.hpp"
#include "mav_dispatcher.hpp"
#include "cmd_executor.hpp"
#include "param_registry.hpp"

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
extern mavlink_system_t   mavlink_system_struct;

extern MavDispatcher      mav_dispatcher;
extern CmdExecutor        cmd_executor;

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

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
void ControllerInit(void){

  uint32_t *sysid;

  /* initial mavlink values */
  param_registry.valueSearch("SYS_ID",  &sysid);

  mavlink_system_struct.sysid  = *sysid;
  mavlink_system_struct.compid = MAV_COMP_ID_ALL;
  mavlink_system_struct.state  = MAV_STATE_BOOT;
  mavlink_system_struct.mode   = MAV_MODE_PREFLIGHT;
  mavlink_system_struct.type   = MAV_TYPE_FIXED_WING;

  /**/
  switch (mavlink_system_struct.type){
  case MAV_TYPE_GROUND_ROVER:
    mav_dispatcher.start(NORMALPRIO);
    cmd_executor.start(NORMALPRIO);
    break;

  case MAV_TYPE_FIXED_WING:
    mav_dispatcher.start(NORMALPRIO);
    cmd_executor.start(NORMALPRIO);
    break;

  default:
    chDbgPanic("This mode is unsupported");
    break;
  }
}

