#include "main.h"

#include "acs.hpp"
#include "mavdbg.hpp"
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
extern mavlink_system_t mavlink_system_struct;

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
acs_status_t ACS::idle_loop(void){
  futaba.failSafe();
  return ACS_STATUS_IDLE;
}

/**
 *
 */
acs_status_t ACS::standby_loop(void){

  if (CH_FAILED == futaba.update(&manual, *acs_timeout))
    futaba.failSafe();

  active_state = ACS_ACTIVE_STATE_MANUAL;
  return ACS_STATUS_DONE;
}

/**
 *
 */
acs_status_t ACS::active_loop(void){

  bool futaba_result = CH_FAILED;

  /* process futaba first */
  if (ACS_ACTIVE_STATE_MANUAL == active_state)
    futaba_result = futaba.update(&manual, *acs_timeout);
  else
    futaba_result = futaba.dryRun(&manual, *acs_timeout);

  if (CH_FAILED == futaba_result){
    active_state = ACS_ACTIVE_STATE_MISSION;
  }
  else{
    switch (manual){
    case MANUAL_SWITCH_MANUAL:
      navigator.UnMemorizeHome();
      mavlink_system_struct.mode &= ~(MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED);
      active_state = ACS_ACTIVE_STATE_MANUAL;
      break;

    case MANUAL_SWITCH_STABILIZE:
      mavlink_system_struct.mode &= ~MAV_MODE_FLAG_GUIDED_ENABLED;
      mavlink_system_struct.mode |=  MAV_MODE_FLAG_STABILIZE_ENABLED;
      active_state = ACS_ACTIVE_STATE_STABILIZE;
      break;

    case MANUAL_SWITCH_MISSION:
      navigator.MemorizeHome();
      mavlink_system_struct.mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
      active_state = ACS_ACTIVE_STATE_MISSION;
      break;

    default:
      chDbgPanic("Futaba. Logic broken in manual detector module");
      break;
    }
  }

  /* cache some current parameters */
  speed_sample = state_vector.vair;
  pitch_sample = state_vector.pitch;
  height_sample = state_vector.alt_baro;

  /* now process ACS */
  switch (active_state){
  case ACS_ACTIVE_STATE_MANUAL:
    stabilizer.dryRun();
    break;

  /* stabilizer loop (currently for testing purposes only) */
  case ACS_ACTIVE_STATE_STABILIZE:
    stabilizer.update(0, pitch_sample, speed_sample);
    //stabilizer.update(0, *pitch_bal, *_acs_speed); // for AHRS-to-plane alignment tests
    break;

  /* automated mission loop */
  case ACS_ACTIVE_STATE_MISSION:
    if (NAVIGATOR_STATUS_DONE != navigator.update()){
      active_state = ACS_ACTIVE_STATE_EMERGENCY;
      navigator.EmergencyLoiter(height_sample);
    }
    break;

  /* emergency loop */
  case ACS_ACTIVE_STATE_EMERGENCY:
    /* раньше тут было только
    navigator.EmergencyLoiter(height_sample);
    но пришлось заменить на очередной костылик
    */
    if(chThdSelf()->p_epending & EVMSK_MISSION_UPDATED){
      if (NAVIGATOR_STATUS_DONE != navigator.update()){
        active_state = ACS_ACTIVE_STATE_EMERGENCY;
        navigator.EmergencyLoiter(height_sample);
      }
    }
    else{
      navigator.EmergencyLoiter(height_sample);
    }
    break;
  }

  /**/
  return ACS_STATUS_DONE;
}

/**
 *
 */
acs_status_t ACS::critical_loop(void){
  chDbgPanic("Unrealized");
  return ACS_STATUS_ERROR;
}

/**
 *
 */
acs_status_t ACS::emergency_loop(void){
  chDbgPanic("Unrealized");
  return ACS_STATUS_ERROR;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
ACS::ACS(Impact &impact, const ACSInput &state_vector,
    SemiautoVector &semiauto_vector, PWMReceiver &pwm_receiver,
    Stabilizer &stabilizer) :
state_vector(state_vector),
stabilizer(stabilizer),
navigator(state_vector, semiauto_vector, stabilizer),
futaba(impact, pwm_receiver),
semiauto_vector(semiauto_vector)
{
  ready = false;
  cycle = 0;
}

/**
 *
 */
void ACS::start(void){
  param_registry.valueSearch("ACS_timeout",   &acs_timeout);
  dV = 0;
  stabilizer.start();
  navigator.start();
  futaba.start();
  futaba.failSafe();
  manual = MANUAL_SWITCH_MANUAL;
  cycle = 0;
  active_state = ACS_ACTIVE_STATE_MANUAL;
  ready = true;
}

/**
 *
 */
void ACS::stop(void){
  ready = false;
  futaba.stop();
  navigator.stop();
  stabilizer.stop();
}

/**
 *
 */
acs_status_t ACS::update(void){

  cycle++;

  switch (mavlink_system_struct.state){
  case MAV_STATE_STANDBY:
    return standby_loop();
    break;

  case MAV_STATE_ACTIVE:
    return active_loop();
    break;

  case MAV_STATE_CRITICAL:
    return critical_loop();
    break;

  case MAV_STATE_EMERGENCY:
    return emergency_loop();
    break;

  default:
    return idle_loop();
    break;
  }

  chDbgPanic("Something broken if your program trapped here");
  return ACS_STATUS_ERROR;
}

/**
 *
 */
MAV_RESULT ACS::takeoff(void){

  /* check underlied system status */
  MAV_RESULT navres = navigator.takeoff();
  if (MAV_RESULT_ACCEPTED != navres)
    return navres;

  /* check system state */
  if (MAV_STATE_STANDBY != mavlink_system_struct.state){
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "ERROR: system must be in STANDBY", MAV_COMP_ID_SYSTEM_CONTROL);
    return MAV_RESULT_TEMPORARILY_REJECTED;
  }
  else{
    mavlink_system_struct.state = MAV_STATE_ACTIVE;
    return MAV_RESULT_ACCEPTED;
  }
}

/**
 *
 */
void ACS::requestSetMode(mavlink_set_mode_t *smp){
  (void)smp;
  mavlink_dbg_print(MAV_SEVERITY_ERROR, "ERROR: mode setting unrealized yet", MAV_COMP_ID_SYSTEM_CONTROL);
}

/**
 *
 */
MAV_RESULT ACS::returnToLaunch(mavlink_command_long_t *clp){
  (void)clp;
  chDbgPanic("Unrealized");
  return MAV_RESULT_FAILED;
}

/**
 *
 */
MAV_RESULT ACS::overrideGoto(mavlink_command_long_t *clp){
  (void)clp;
  chDbgPanic("Unrealized");
  return MAV_RESULT_FAILED;
}

/**
 *
 */
MAV_RESULT ACS::emergencyGotoLand(mavlink_command_long_t *clp){
  (void)clp;
  chDbgPanic("Unrealized");
  return MAV_RESULT_FAILED;
}

/**
 *
 */
MAV_RESULT ACS::setCurrentMission(mavlink_mission_set_current_t *scp){
  (void)scp;
  chDbgPanic("Unrealized");
  return MAV_RESULT_FAILED;
}

/**
 *
 */
void ACS::manualControl(mavlink_manual_control_t *mcp){
  (void)mcp;
  //chDbgPanic("Unrealized");
}

/**
 *
 */
void ACS::EmergencyReturn(void){
  navigator.EmergencyReturn();
}

bool ACS::IsGoingHome() {
  return navigator.IsGoingHome();
}

bool ACS::IsEmergencyConnectionLost() {
  return navigator.IsEmergencyConnectionLost();
}
/**
 *
 */
navigator_status_t ACS::EmergencyJump(uint16_t seq){
  return navigator.EmergencyJump(seq);
}

/**
 *
 */
navigator_status_t ACS::EmergencyConnectionLost(void){
  return navigator.EmergencyConnectionLost();
}

navigator_status_t ACS::EmergencyResetConnectionLost(void){
  return navigator.EmergencyResetConnectionLost();
}


/**
 *
 */
ACS::State ACS::getCurrentState() {

  if (this->active_state == ACS_ACTIVE_STATE_MANUAL)
    return ACS::State::Manual;

  if (this->semiauto_vector.enabled == true)
    return ACS::State::Automated;

  if (this->active_state == ACS_ACTIVE_STATE_MISSION)
    return ACS::State::Mission;

  return ACS::State::Undefined;
}


void ACS::setCurrentState(ACS::State newState) {
  if (newState == ACS::State::Automated) {
    this->semiauto_vector.enabled = true;
  }
  else {
    this->semiauto_vector.enabled = false;
  }
}

void ACS::setHeading(float heading) {
  this->semiauto_vector.hdg = heading;
}

void ACS::setAltitude(float altitude) {
  this->semiauto_vector.alt = altitude;
}

void ACS::setAirSpeed(float speed) {
  this->semiauto_vector.air_speed = speed;
}


