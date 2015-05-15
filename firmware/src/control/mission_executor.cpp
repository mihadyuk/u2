#include "main.h"

#include "mission_executor.hpp"
#include "waypoint_db.hpp"
#include "mav_dbg.hpp"

using namespace chibios_rt;
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
uint16_t MissionExecutor::current_cmd(void) {
  return segment[1].command;
}

/**
 *
 */
void MissionExecutor::what_to_do_here(void) {

  switch(current_cmd()) {
  case (MAV_CMD_NAV_LOITER_UNLIM || MAV_CMD_NAV_LOITER_TURNS ||  MAV_CMD_NAV_LOITER_TIME):
    /* we must loter here according to mission plan */
    mavlink_dbg_print(MAV_SEVERITY_INFO, "ACS: start loitering", MAV_COMP_ID_SYSTEM_CONTROL);
    break;

  case MAV_CMD_NAV_WAYPOINT:
    /* this is regular waypoint. Just go to next item */
    load_mission_item();
    break;

  default:
    /* do not know hot to handle it. Just got to next one */
    load_mission_item();
    break;
  }
}

/**
 *
 */
uint16_t MissionExecutor::do_jump(void){
  return 0;
}

/**
 *
 */
void MissionExecutor::fake_last_point(void) {
  segment[2] = segment[1];
}

/**
 *
 */
void MissionExecutor::load_mission_item(void) {

  bool load_status = OSAL_FAILED;
  uint16_t next;

  mavlink_mission_item_t *mi_next = &this->segment[2];

  segment[0] = segment[1];
  segment[1] = segment[2];

  if (wpdb.getCount() <= (mi_next->seq + 1)){ // no more items
    /* if we fall here than last mission was not 'land'. System do not know
     * what to do so start unlimited loitering */
    //navigator.start_loiter();
  }
  else {
    next = mi_next->seq + 1;

    /* try to load */
    load_status = wpdb.read(mi_next, next);
    if (OSAL_SUCCESS != load_status){
      /* something goes wrong with waypoint loading */
      //navigator.start_loiter();
    }
  }

  broadcast_mission_current(segment[1].seq);
}

/**
 *
 */
void MissionExecutor::maneuver(void) {
  return;
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
acs_in(acs_in)
{
  state = MissionState::uninit;
  memset(segment, 0, sizeof(segment));
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
  memset(segment, 0, sizeof(segment));
}

/**
 *
 */
bool MissionExecutor::takeoff(void) {

  if (wpdb.getCount() < 3) {
    mavlink_dbg_print(MAV_SEVERITY_INFO, "ACS: mission must be at least 3 WP long", MAV_COMP_ID_SYSTEM_CONTROL);
    return OSAL_FAILED;
  }
  else {
    wpdb.read(&this->segment[0], 0);
    wpdb.read(&this->segment[1], 1);
    wpdb.read(&this->segment[2], 2);

    state = MissionState::navigate;

    return OSAL_SUCCESS;
  }
}

/**
 *
 */
MissionStatus MissionExecutor::update(float dT) {

  osalDbgCheck(MissionState::uninit != state);

  (void)dT;

  switch (state) {
  case MissionState::idle:
    break;
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
  return;
}


