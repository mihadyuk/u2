#include "main.h"
#include "global_flags.h"
#include "message.hpp"
#include "param_registry.hpp"
#include "attitude_unit.hpp"
#include "cmd_executor.hpp"
#include "baro_unit.hpp"
#include "mavdbg.hpp"
#include "mavlocal.hpp"

#if defined(ENABLE_STANAG)
#include "stanagMainService.hpp"
#include "stanagGoes.hpp"
#endif

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
extern mavlink_system_t mavlink_system_struct;
extern mavlink_command_ack_t mavlink_out_command_ack_struct;
extern mavlink_command_long_t mavlink_in_command_long_struct;

extern EventSource event_mavlink_in_command_long;

#if defined(ENABLE_STANAG)
extern stanag::MainService stanagMain;
#endif
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
 * helper funcion
 */
static void cmd_confirm(uint8_t result, uint16_t cmd){
  mavlink_out_command_ack_struct.result = result;
  mavlink_out_command_ack_struct.command = cmd;
  CommandAckSend(&mavlink_out_command_ack_struct, MAV_COMP_ID_SYSTEM_CONTROL);
}

/**
 * NOTE: Calibration of gyros and magnetometers can not be run simulatiously
 */
enum MAV_RESULT CmdExecutor::cmd_calibration_handler(mavlink_command_long_t *cl){
 /* Trigger calibration. This command will be only accepted if in pre-flight mode.
  * | Gyro calibration: 0: no, 1: yes
  * | Magnetometer calibration: 0: no, 1: yes
  * | Ground pressure: 0: no, 1: yes
  * | Radio calibration: 0: no, 1: yes
  * | Empty| Empty| Empty|  */

  bool result;

  if (mavlink_system_struct.mode != MAV_MODE_PREFLIGHT)
    return MAV_RESULT_TEMPORARILY_REJECTED;

  /* Gyro */
  if (cl->param1 == 1){
    result = attitude_unit.trigCalibrateGyro();
    if (CH_SUCCESS == result)
      return MAV_RESULT_ACCEPTED;
    else
      return MAV_RESULT_FAILED;
  }

  /* Magnetometer */
  else if (cl->param2 == 1){
    result = attitude_unit.trigCalibrateMag();
    if (CH_SUCCESS == result)
      return MAV_RESULT_ACCEPTED;
    else
      return MAV_RESULT_FAILED;
  }

  /* Barometer */
  else if (cl->param3 == 1){
    result = TrigCalibrateBaro();
    if (CH_SUCCESS == result)
      return MAV_RESULT_ACCEPTED;
    else
      return MAV_RESULT_FAILED;
  }

  /* unknown */
  else
    return MAV_RESULT_DENIED;
}

/**
 *
 */
enum MAV_RESULT CmdExecutor::cmd_reboot_shutdown_handler(mavlink_command_long_t *cl){
 /* Request the reboot or shutdown of system components.|
  * 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.|
  * 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.
  * | Reserved| Reserved| Empty| Empty| Empty|  */

  if (mavlink_system_struct.mode != MAV_MODE_PREFLIGHT)
    return MAV_RESULT_TEMPORARILY_REJECTED;

  /* you can perform power operation on autopilot OR computer, not both at once */
  if (cl->param1 == 1.0f){/* reboot */
    NVIC_SystemReset();
    return MAV_RESULT_UNSUPPORTED;
  }
  else if (cl->param1 == 2.0f){/* shutdown */
    //setGlobalFlag(GlobalFlags.sighalt);
    return MAV_RESULT_UNSUPPORTED;
  }

  /**/
  if (cl->param2 == 1.0f){/* reboot */
    return MAV_RESULT_UNSUPPORTED;
  }
  else if (cl->param2 == 2.0f){/* shutdown */
    return MAV_RESULT_UNSUPPORTED;
  }

  /* default */
  return MAV_RESULT_UNSUPPORTED;
}

/**
 *
 */
enum MAV_RESULT CmdExecutor::cmd_preflight_storage_handler(mavlink_command_long_t *cl){
  bool_t status = CH_FAILED;

  /* Mavlink protocol clames "This command will be only accepted if
   * in pre-flight mode." But in our case using MAV_STATE_STANDBY looks
   * more suitable. */

  //if (mavlink_system_struct.mode != MAV_MODE_PREFLIGHT)
  //  return MAV_RESULT_TEMPORARILY_REJECTED;

//  if (mavlink_system_struct.state != MAV_STATE_STANDBY)
//    return MAV_RESULT_TEMPORARILY_REJECTED;

  mavlink_dbg_print(MAV_SEVERITY_INFO, "eeprom operation started", MAV_COMP_ID_SYSTEM_CONTROL);
  if (cl->param1 == 0)
    status = param_registry.load();
  else if (cl->param1 == 1)
    status = param_registry.saveAll();

  if (status != CH_SUCCESS){
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "ERROR: eeprom operation failed", MAV_COMP_ID_SYSTEM_CONTROL);
    return MAV_RESULT_FAILED;
  }
  else{
    mavlink_dbg_print(MAV_SEVERITY_INFO, "OK: eeprom operation success", MAV_COMP_ID_SYSTEM_CONTROL);
    return MAV_RESULT_ACCEPTED;
  }
}

/**
 *
 */
enum MAV_RESULT CmdExecutor::cmd_nav_roi_handler(mavlink_command_long_t *cl){
  (void)cl;
#if defined(ENABLE_STANAG)
  stanag::Goes *goes = stanagMain.GetGoes();
  if (goes && cl) {

    goes->ChangePosition(cl->param5/10000000.0f,
                         cl->param6/10000000.0f,
                         cl->param7);
  }
#else
  (void)cl;
#endif

//#error "past goes change stare point here.
//  Use griffon xml to figure out wich params are used for starepoint. it seems 5 6 7 params"
  return MAV_RESULT_ACCEPTED;
}

/**
 * Looks like duplicated functionality of SET_MODE message
 */
enum MAV_RESULT CmdExecutor::cmd_do_set_mode_handler(mavlink_command_long_t *cl){
  /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|  */
  (void)cl;
  return MAV_RESULT_DENIED;
}


/**
 * Parses and executes commands. Commands can be both from ground or
 * sent by ACS from mission list.
 */
void CmdExecutor::executeCmd(mavlink_command_long_t *clp){
  enum MAV_RESULT result = MAV_RESULT_DENIED;

  /* all this flags defined in MAV_CMD enum */
  switch(clp->command){
  /*
   *
   */
  case MAV_CMD_DO_SET_MODE:
    result = cmd_do_set_mode_handler(clp);
    cmd_confirm(result, clp->command);
    break;

  /*
   * (re)calibrate
   */
  case MAV_CMD_PREFLIGHT_CALIBRATION:
    result = cmd_calibration_handler(clp);
    cmd_confirm(result, clp->command);
    break;

  /*
   *
   */
  case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
    result = cmd_reboot_shutdown_handler(clp);
    cmd_confirm(result, clp->command);
    break;

  /*
   * Data (storing to)/(loding from) EEPROM
   */
  case MAV_CMD_PREFLIGHT_STORAGE:
    result = cmd_preflight_storage_handler(clp);
    cmd_confirm(result, clp->command);
    break;

  /*
   * take off handler
   */
  case (MAV_CMD_NAV_TAKEOFF):
    result = acs.takeoff();
    cmd_confirm(result, clp->command);
    break;

  /*
   *
   */
  case (MAV_CMD_NAV_RETURN_TO_LAUNCH):
    result = acs.returnToLaunch(clp);
    cmd_confirm(result, clp->command);
    break;

  /*
   *
   */
  case (MAV_CMD_NAV_LAND):
    result = acs.emergencyGotoLand(clp);
    cmd_confirm(result, clp->command);
    break;

  /*
   *
   */
  case (MAV_CMD_OVERRIDE_GOTO):
    result = acs.overrideGoto(clp);
    cmd_confirm(result, clp->command);
    break;

  /*
   *
   */
  case (MAV_CMD_NAV_ROI):
    result = cmd_nav_roi_handler(clp);;
    cmd_confirm(result, clp->command);
    break;

  /*
   *
   */
  default:
    cmd_confirm(MAV_RESULT_UNSUPPORTED, clp->command);
    break;
  }
}

/**
 * Command processing thread.
 */
msg_t CmdExecutor::main(void){
  this->setName("CmdExecutor");
  eventmask_t evt = 0;
  struct EventListener el_command_long;
  chEvtRegisterMask(&event_mavlink_in_command_long, &el_command_long, EVMSK_MAVLINK_IN_COMMAND_LONG);

  /* wait modems */
  while(GlobalFlags.messaging_ready == 0)
    chThdSleepMilliseconds(50);

  /* main loop */
  while(!chThdShouldTerminate()){
    evt = chEvtWaitOneTimeout(EVMSK_MAVLINK_IN_COMMAND_LONG, MS2ST(50));

    switch (evt){
    case EVMSK_MAVLINK_IN_COMMAND_LONG:
      executeCmd(&mavlink_in_command_long_struct);
      break;

    default:
      break;
    }
  }

  chEvtUnregister(&event_mavlink_in_command_long, &el_command_long);
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
CmdExecutor::CmdExecutor(ACS &acs, AttitudeUnit &attitude_unit):
acs(acs),
attitude_unit(attitude_unit)
{
//  chDbgCheck((NULL != acs) && (NULL != attitude_unit), "");
//  this->acs = acs;
//  this->attitude_unit = attitude_unit;
  return;
}
