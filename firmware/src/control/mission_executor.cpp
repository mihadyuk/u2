#include "main.h"

#include "mission_executor.hpp"
#include "waypoint_db.hpp"

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
void MissionExecutor::maneuver(void) {
  return;
}

/**
 *
 */
void MissionExecutor::navigate(void) {
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
MissionExecutor::MissionExecutor(void) {
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
void MissionExecutor::launch(void) {

  wpdb.read(&this->segment[0], 0);
  wpdb.read(&this->segment[1], 1);
  wpdb.read(&this->segment[2], 2);

  state = MissionState::navigate;
}

/**
 *
 */
void MissionExecutor::update(StabInput &stab, float dT) {

  osalDbgCheck(MissionState::uninit != state);

  (void)stab;
  (void)dT;

  switch (state) {
  case MissionState::navigate:
    this->navigate();
    break;

  case MissionState::maneuver:
    this->maneuver();
    break;

  /**/
  default:
    break;
  }
}

/**
 *
 */
void MissionExecutor::setHome(void) {
  return;
}


