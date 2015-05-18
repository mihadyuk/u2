#include "main.h"

#include "mission_executor.hpp"
#include "waypoint_db.hpp"
#include "mav_dbg.hpp"
#include "navigator_types.hpp"

using namespace chibios_rt;
using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define WP_RADIUS     param1

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_system_t                 mavlink_system_struct;
extern mavlink_mission_current_t        mavlink_out_mission_current_struct;
extern mavlink_mission_item_reached_t   mavlink_out_mission_item_reached_struct;

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
void MissionExecutor::broadcast_mission_current(uint16_t seq) {
  mavlink_out_mission_current_struct.seq = seq;
}

/**
 *
 */
void MissionExecutor::broadcast_mission_item_reached(uint16_t seq) {
  mavlink_out_mission_item_reached_struct.seq = seq;
}

/**
 *
 */
bool MissionExecutor::load_next_mission_item(void) {

  bool load_status = OSAL_FAILED;
  uint16_t next;

  prev = trgt;
  trgt = third;

  if (wpdb.getCount() <= (third.seq + 1)){ // no more items
    /* if we fall here than last mission was not 'land'. System do not know
     * what to do so start unlimited loitering */
    third = trgt;
    third.x += 0.0001;
    third.y += 0.0001;
    third.command = MAV_CMD_NAV_LOITER_UNLIM;
    state = MissionState::completed;
  }
  else {
    next = third.seq + 1;
    load_status = wpdb.read(&third, next);
  }

  if (OSAL_SUCCESS == load_status) {
    state = MissionState::navigate;
    broadcast_mission_current(trgt.seq);
  }
  else
    state = MissionState::error;

  return load_status;
}

/**
 *
 */
static void nav_out_to_acs_in(const NavOut<float> &nav_out, ACSInput &acs_in) {
  acs_in.ch[ACS_INPUT_dZ] = nav_out.xtd;
}

/**
 * @brief   Detects waypoint reachable status.
 * @details Kind of hack. It increases effective radius if
 *          crosstrack error more than R/1.5
 */
bool MissionExecutor::wp_reached(const NavOut<float> &nav_out) {

  float R = nav_out.xtd * 1.5;

  if (R < trgt.WP_RADIUS)
    R = trgt.WP_RADIUS;
  /* else branch unneeded here because R already contains correct value */

  return nav_out.dist < R;
}

/**
 *
 */
void MissionExecutor::navigate(void) {

  NavIn<float> nav_in(acs_in.ch[ACS_INPUT_lat], acs_in.ch[ACS_INPUT_lon]);
  NavOut<float> nav_out = navigator.update(nav_in);

  nav_out_to_acs_in(nav_out, this->acs_in);

  if (wp_reached(nav_out)) {
    load_next_mission_item();
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
MissionExecutor::MissionExecutor(ACSInput &acs_in) :
state(MissionState::uninit),
acs_in(acs_in) {

  /* fill home point with invalid data. This denotes it uninitialized. */
  memset(&home, 0xFF, sizeof(home));
  home.seq = -1;
}

/**
 *
 */
void MissionExecutor::start(void) {
  state = MissionState::idle;
}

/**
 *
 */
void MissionExecutor::stop(void) {
  state = MissionState::uninit;
}

/**
 *
 */
void MissionExecutor::artificial_takeoff_point(void) {

  memset(&prev, 0, sizeof(prev));

  prev.x = acs_in.ch[ACS_INPUT_lat];
  prev.y = acs_in.ch[ACS_INPUT_lon];
  prev.z = acs_in.ch[ACS_INPUT_alt];
  prev.command = MAV_CMD_NAV_TAKEOFF;
  prev.frame = MAV_FRAME_GLOBAL;
  prev.seq = -1;

  /* store launch point */
  launch = prev;

  /* set launch point as home if it was not set yet from ground */
  if (uint16_t(-1) == home.seq) {
    home = launch;
    home.command = MAV_CMD_NAV_WAYPOINT;
  }
}


/**
 * @brief   Samples current coordinates as prev WP than append
 *          first 2 points from mission.
 */
bool MissionExecutor::takeoff(void) {
  bool read_status1 = OSAL_FAILED;
  bool read_status2 = OSAL_FAILED;

  if (wpdb.getCount() < 2) {
    mavlink_dbg_print(MAV_SEVERITY_INFO, "ACS: mission must be at least 2 WP long", MAV_COMP_ID_SYSTEM_CONTROL);
    return OSAL_FAILED;
  }
  else {
    artificial_takeoff_point();
    read_status1 = wpdb.read(&this->trgt,  0);
    read_status2 = wpdb.read(&this->third, 1);
    if ((OSAL_SUCCESS != read_status1) || (OSAL_SUCCESS != read_status2))
      return OSAL_FAILED;
    else {
      NavLine<float> line(prev.x, prev.y, trgt.x, trgt.y);
      navigator.loadLine(line);
      state = MissionState::navigate;
      return OSAL_SUCCESS;
    }
  }
}

/**
 *
 */
MissionState MissionExecutor::update(void) {

  osalDbgCheck(MissionState::uninit != state);

  switch (state) {
  case MissionState::navigate:
    this->navigate();
    break;

  default:
    break;
  }

  return this->state;
}

/**
 *
 */
void MissionExecutor::setHome(void) {

  setHome(acs_in.ch[ACS_INPUT_lat], acs_in.ch[ACS_INPUT_lon], acs_in.ch[ACS_INPUT_alt]);
}

/**
 *
 */
void MissionExecutor::setHome(float lat, float lon, float alt) {

  memset(&home, 0, sizeof(home));

  home.x = lat;
  home.y = lon;
  home.z = alt;
  home.command = MAV_CMD_NAV_WAYPOINT;
  home.frame = MAV_FRAME_GLOBAL;
  home.seq = -1;
}

/**
 *
 */
bool MissionExecutor::loadNext(void) {
  return this->load_next_mission_item();
}

/**
 *
 */
void MissionExecutor::goHome(void) {
  return;
}

/**
 *
 */
void MissionExecutor::returnToLaunch(void) {
  return;
}

/**
 *
 */
bool MissionExecutor::jumpTo(uint16_t seq) {
  (void)seq;

  return OSAL_FAILED;
}

/**
 *
 */
uint16_t MissionExecutor::getTrgtCmd(void) {
  return trgt.command;
}
