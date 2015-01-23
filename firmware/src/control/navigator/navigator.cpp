#include "main.h"

#include "navigator.hpp"
#include "mavdbg.hpp"
#include "mavlocal.hpp"
#include "logger.hpp"
#include "waypoint_db.hpp"
#include "param_registry.hpp"
#include "message.hpp"

#include "fileService.hpp"
#include "stanagMainService.hpp"
#include "stanagGoes.hpp"


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
extern mavlink_system_t                 mavlink_system_struct;
extern mavlink_mission_current_t        mavlink_out_mission_current_struct;
extern mavlink_mission_item_reached_t   mavlink_out_mission_item_reached_struct;
extern mavlink_griffon_target_vector_t  mavlink_out_griffon_target_vector_struct;

extern WpDB wpdb;

extern EventSource event_mission_updated;

extern stanag::MainService stanagMain;
//extern fileServices::FileService fileService;
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
void Navigator::broadcast_mission_current(uint16_t seq){
  mavlink_out_mission_current_struct.seq = seq;
  MissionCurrentSend(&mavlink_out_mission_current_struct, MAV_COMP_ID_SYSTEM_CONTROL);
#if defined(STANAG_ENABLE_GOES_CONTROL_ACS) && defined(ENABLE_STANAG)
  this->handlePayloadActionWaypoint();
#endif
#if defined(STANAG_ENABLE_MISSION_ACS) && defined(ENABLE_STANAG)
  stanagMain.UpdateCurrentWaypoint(seq + 1);
#endif
  log_write_schedule(MAVLINK_MSG_ID_MISSION_CURRENT, NULL, 0);
}

/**
 *
 */
void Navigator::broadcast_mission_item_reached(uint16_t seq){
  mavlink_out_mission_item_reached_struct.seq = seq;
  MissionItemReachedSend(&mavlink_out_mission_item_reached_struct, MAV_COMP_ID_SYSTEM_CONTROL);
#if defined(STANAG_ENABLE_MISSION_ACS) && defined(ENABLE_STANAG)
  stanagMain.UpdateReachedWaypoint(seq + 1);
#endif
  log_write_schedule(MAVLINK_MSG_ID_MISSION_ITEM_REACHED, NULL, 0);
}

/**
 *
 */
void Navigator::what_to_do_here(void){
  switch(mi.command){
  case (MAV_CMD_NAV_LOITER_UNLIM || MAV_CMD_NAV_LOITER_TURNS ||  MAV_CMD_NAV_LOITER_TIME):
    /* we must loter here according to mission plan */
    mavlink_dbg_print(MAV_SEVERITY_INFO, "ACS: start loitering", MAV_COMP_ID_SYSTEM_CONTROL);
    loiter_halfturns = 0;
    loiter_timestamp = chTimeNow();
    state = NAVIGATOR_STATE_LOITER;
    break;

  case MAV_CMD_NAV_WAYPOINT:
    /* this is regular waypoint. Just go to next item */
    state = NAVIGATOR_STATE_LOAD_MISSION_ITEM;
    break;

  case MAV_CMD_NAV_TAKEOFF:
    /* rover successfully navigated to takeoff point. Go to next item */
    state = NAVIGATOR_STATE_LOAD_MISSION_ITEM;
    break;

  case MAV_CMD_NAV_LAND:
    /* landing point reached. Go to idle state */
    state = NAVIGATOR_STATE_IDLE;
    mavlink_system_struct.mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
    mavlink_system_struct.state = MAV_STATE_STANDBY;
    mavlink_dbg_print(MAV_SEVERITY_INFO, "ACS: Landed", MAV_COMP_ID_SYSTEM_CONTROL);
    break;

  case MAV_CMD_DO_JUMP:
    EmergencyJump(roundf(mi.R));
    break;

  default:
    /* do not know hot to handle it. Just got to next one */
    state = NAVIGATOR_STATE_LOAD_MISSION_ITEM;
    break;
  }
}

/**
 *
 */
uint16_t Navigator::do_jump(void){

  if (!jump_active){
    jump_cycle = mi.param2;
    jump_active = true;
  }

  if (jump_cycle > 0){
    jump_cycle--;
    return mi.R;
  }
  else{
    jump_active = false;
    return mi.seq + 1;
  }
}

/**
 *
 */
navigator_status_t Navigator::loop_load_mission_item(void){

  bool load_status = CH_FAILED;
  uint16_t next;

  /* special hack for emergency loitering under home point */
  if (true == going_home){
    state = NAVIGATOR_STATE_LOITER;
    return NAVIGATOR_STATUS_NO_MISSION;
  }

  mi_prev = mi;

  if (wpdb.loadCount() <= (mi.seq + 1)){ // no more items
    /* if we fall here than last mission was not 'land'. System do not know
     * what to do so start unlimited loitering */
    state = NAVIGATOR_STATE_LOITER;
    return NAVIGATOR_STATUS_NO_MISSION;
  }
  else
    next = mi.seq + 1;

  /* try to load */
  load_status = wpdb.load(&mi, next);
  if (CH_SUCCESS != load_status){
    /* something goes wrong with waypoint loading */
    state = NAVIGATOR_STATE_LOITER;
    return NAVIGATOR_STATUS_ERROR;
  }

  if (MAV_CMD_DO_JUMP == mi.command){
    next = do_jump();
    load_status = wpdb.load(&mi, next);
    if (CH_SUCCESS != load_status){
      /* something goes wrong with waypoint loading */
      state = NAVIGATOR_STATE_LOITER;
      return NAVIGATOR_STATUS_ERROR;
    }
  }

  broadcast_mission_current(mi.seq);
  sphere.updatePoints(mi_prev.x, mi_prev.y, mi.x, mi.y);
  state = NAVIGATOR_STATE_NAVIGATE_TO_WAYPOINT;
  return NAVIGATOR_STATUS_DONE;
}

/**
 *
 */
navigator_status_t Navigator::reload_mission(void){

  /* only reload mission can disable going home */
  going_home = false;

  /* DIRTY workaround! Most safe approach when mission incomplete. */
  if (wpdb.loadCount() < 2){
    return NAVIGATOR_STATUS_MISSION_COMPLETED;
  }

  /* update current crsline */
  if (CH_FAILED == wpdb.load(&mi_prev, 0) || CH_FAILED == wpdb.load(&mi, 1)){
    return NAVIGATOR_STATUS_ERROR;
  }
  else{
    broadcast_mission_current(mi.seq);
    return NAVIGATOR_STATUS_DONE;
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
Navigator::Navigator(const StateVector &state_vector,
    const SemiautoVector &semiauto_vector, Stabilizer &stabilizer) :
state_vector(state_vector),
semiauto_vector(semiauto_vector),
stabilizer(stabilizer),
jump_active(false),
std_maneur_active(false),
loiter_start_time(0),
going_home(false),
semLock(false)
#if defined(STANAG_ENABLE_GOES_CONTROL_ACS)
, route(fileServices::FileService::GetFileSystem())
#endif
{
  state = NAVIGATOR_STATE_UNINIT;
}

/**
 *
 */
navigator_status_t Navigator::update(void){

  eventmask_t evt = 0;

  chDbgCheck(state != NAVIGATOR_STATE_UNINIT, "Navigator in wrong state");

  /* FIXME: выбросить нахуй это говно и переделать всю логику следования по маршруту */
  evt = chEvtGetAndClearFlags(&el_mission_updated);
  if (NAVIGATOR_STATE_NAVIGATE_TO_WAYPOINT == state ||
             NAVIGATOR_STATE_PASS_WAYPOINT == state ||
                    NAVIGATOR_STATE_LOITER == state){
    if (EVMSK_MISSION_UPDATED == (evt & EVMSK_MISSION_UPDATED)){
      if (NAVIGATOR_STATUS_DONE != reload_mission())
        state = NAVIGATOR_STATE_LOITER;
      else
        state = NAVIGATOR_STATE_NAVIGATE_TO_WAYPOINT;
    }
  }

  mavlink_out_griffon_target_vector_struct.lat = this->mi.x;
  mavlink_out_griffon_target_vector_struct.lon = this->mi.y;
  mavlink_out_griffon_target_vector_struct.alt = this->mi.z;
  mavlink_out_griffon_target_vector_struct.wp_current = this->mi.seq;
  //mavlink_out_griffon_target_vector_struct.wp_count = wpdb.loadCount();

  /* FIXME: Очередной ебучий говнокод для агата */
  if (semiauto_vector.enabled && this->going_home == false){
    return semiauto_govnocode();
  }
  else{
    semiauto_first_call = 0xFFFFFFFF;
  }

  /* nomal code start */
  switch(state){
  case NAVIGATOR_STATE_IDLE:
    return NAVIGATOR_STATUS_IDLE;
    break;

  case NAVIGATOR_STATE_NAVIGATE_TO_WAYPOINT:
    return loop_navigate();
    break;

  case NAVIGATOR_STATE_LOITER:
    return loop_loiter();
    break;

  case NAVIGATOR_STATE_PASS_WAYPOINT:
    return loop_passwp();
    break;

  case NAVIGATOR_STATE_TAKEOFF:
    return loop_takeoff();
    break;

  case NAVIGATOR_STATE_LOAD_MISSION_ITEM:
    return loop_load_mission_item();
    break;

  case NAVIGATOR_STATE_LAND:
    return loop_land();
    break;

#if 0
  case NAVIGATOR_STATE_CONNECTION_LOST:
    return loop_connection_lost();
    break;
#endif

  case NAVIGATOR_STATE_UNINIT:
    chDbgPanic("Navigator in wrong state");
    break;
  }

  return NAVIGATOR_STATUS_DONE;
}

/**
 *
 */
void Navigator::start(void){

  param_registry.valueSearch("ACS_k_dZ",      &k_dZ);
  param_registry.valueSearch("ACS_k_dHead",   &k_dHead);
  param_registry.valueSearch("ACS_k_dH",      &k_dH);
  param_registry.valueSearch("ACS_k_dG",      &k_dG);
  param_registry.valueSearch("ACS_roll_lim",  &roll_lim);
  param_registry.valueSearch("ACS_pitch_lim", &pitch_lim);
  param_registry.valueSearch("ACS_dZ_lim",    &dZ_lim);
  param_registry.valueSearch("ACS_pitch_bal", &pitch_bal);
  param_registry.valueSearch("ACS_dZmax",     &dZmax);

  chEvtRegisterMask(&event_mission_updated, &el_mission_updated, EVMSK_MISSION_UPDATED);

  state = NAVIGATOR_STATE_IDLE;
}

/**
 *
 */
void Navigator::stop(void){
  state = NAVIGATOR_STATE_UNINIT;
}

/**
 *
 */
MAV_RESULT Navigator::takeoff(void){

  MAV_RESULT ret = MAV_RESULT_FAILED;

//#if TAKE_OFF_ANTI_MOTHERFUCKER_PROTECTION
  /* check navigator state */
  if (NAVIGATOR_STATE_IDLE != state){
    mavlink_dbg_print(MAV_SEVERITY_ERROR,
                      "ERROR: navigator must be in IDLE state",
                      MAV_COMP_ID_SYSTEM_CONTROL);
    ret = MAV_RESULT_TEMPORARILY_REJECTED;
    goto FAIL;
  }

  /* TODO: system disarmed check */
  //if (!(mavlink_system_struct.mode & MAV_MODE_FLAG_SAFETY_ARMED)){
  //  mavlink_dbg_print(MAV_SEVERITY_ERROR, "ERROR: motors disarmed", MAV_COMP_ID_SYSTEM_CONTROL);
  //  return MAV_RESULT_TEMPORARILY_REJECTED;
  //}

  /* empty mission discoraged */
  if (0 == wpdb.loadCount()){
    mavlink_dbg_print(MAV_SEVERITY_ERROR,
                      "ERROR: mission empty",
                      MAV_COMP_ID_SYSTEM_CONTROL);
    ret = MAV_RESULT_DENIED;
    goto FAIL;
  }

  /* try to load start mission item to RAM */
  if (CH_SUCCESS != wpdb.load(&mi, 0)){
    mavlink_dbg_print(MAV_SEVERITY_ERROR,
                      "ERROR: can not load first mission item",
                      MAV_COMP_ID_SYSTEM_CONTROL);
    ret = MAV_RESULT_FAILED;
    goto FAIL;
  }

  /**/
  if (mi.command != MAV_CMD_NAV_TAKEOFF){
    mavlink_dbg_print(MAV_SEVERITY_ERROR,
                      "ERROR: first mission item must be 'take_off'",
                      MAV_COMP_ID_SYSTEM_CONTROL);
    ret = MAV_RESULT_DENIED;
    goto FAIL;
  }

  /* can not fly without GPS */
//  if (state_vector.gpsfix < 2){    /* can not fly without GPS */
//    mavlink_dbg_print(MAV_SEVERITY_ERROR,
//                      "ERROR: no GPS fix",
//                      MAV_COMP_ID_SYSTEM_CONTROL);
//    ret = MAV_RESULT_TEMPORARILY_REJECTED;
//    goto FAIL;
//  }

  /* catch starting point for trajectory constructing */
  sphere.updatePoints(state_vector.lat, state_vector.lon, mi.x, mi.y);
  mi_prev = mi; // hack to fill all other fields correctly
  mi_prev.x = state_vector.lat;
  mi_prev.y = state_vector.lon;
  mi_prev.z = state_vector.alt;

  mavlink_dbg_print(MAV_SEVERITY_INFO,
                    "ACS: Taking off",
                    MAV_COMP_ID_SYSTEM_CONTROL);
  state = NAVIGATOR_STATE_TAKEOFF;
  broadcast_mission_current(mi.seq);
  return MAV_RESULT_ACCEPTED;

FAIL:
#if TAKE_OFF_ANTI_MOTHERFUCKER_PROTECTION
  return ret;
#else
  (void)ret;
  /* suppress all errors and just start loitering */
  state = NAVIGATOR_STATE_LOITER;
  return MAV_RESULT_ACCEPTED;
#endif /* TAKE_OFF_ANTI_MOTHERFUCKER_PROTECTION */
}

/**
 *
 */
#if defined(STANAG_ENABLE_GOES_CONTROL_ACS)
void Navigator::_handlePayloadActionWaypoint() {

  Goes *goes = stanagMain.GetGoes();
  if (!goes) {
    /* goes control is not allowed. return.*/
    return;
  }

  if (this->route.OpenForRead(stanagMain.GetActiveRouteName()) == false) {
    return;
  }

  /* look for payload action waypoint.*/
  PayloadActionWaypoint *wpt = new PayloadActionWaypoint();
  if (!wpt) {
    this->route.Close();
    return;
  }
  uint16_t currentWaypointNumber = wpdb.getCurrentStanagWaypointNumber();
  if (this->route.FindWaypoint(currentWaypointNumber, *wpt) == false) {
    /* waypoint hasn't been found. Just return and no action to control goes.*/
    delete wpt;
    this->route.Close();
    return;
  }
  /* perform action for goes.*/

  /* handle enabling/disabling goes.*/
  uint8_t sensor1Mode = 0;
  if (wpt->GetSensor1Mode(sensor1Mode) == true) {
    if (sensor1Mode == 0)
      goes->Stop();
    else
      goes->Start();
  }

  /* set zoom for goes.*/
  uint8_t zoom  = 1;
  if (wpt->GetGoesZoom(zoom) == true) {
    goes->SetZoom(zoom);
  }

  /* update starepoint position.*/
  float lat     = 0.0;
  float longit  = 0.0;
  float alt     = 0.0;
  if (wpt->GetStarepointLatitude(lat)     &&
      wpt->GetStarepointLongitude(longit) &&
      wpt->GetStarepointAltitude(alt)) {
    goes->ChangePositionRad(lat, longit, alt);
  }

  delete wpt;
  this->route.Close();
  return;
}

void Navigator::handlePayloadActionWaypoint() {

  MissionService::lockWriteMission.wait();
  this->_handlePayloadActionWaypoint();
  MissionService::lockWriteMission.signal();
}

#endif


