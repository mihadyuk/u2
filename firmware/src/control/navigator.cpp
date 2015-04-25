#include "main.h"

#include "navigator.hpp"
#include "param_registry.hpp"

namespace control {

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

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
Navigator::Navigator(void) {
  return;
}

/**
 *
 */
NavigatorStatus Navigator::update(ACSInput &acs_in) {
  osalDbgCheck(ready);

  sphere.crosstrack(acs_in.ch[ACS_INPUT_lat], acs_in.ch[ACS_INPUT_lon],
                    &acs_in.ch[ACS_INPUT_dZ], nullptr);

  return NavigatorStatus::navigate;
}

/**
 *
 */
void Navigator::load_segment(const mavlink_mission_item_t *segment) {

  for (size_t i=0; i<NAVIGATOR_SEGMENT_LEN; i++)
    this->segment[i] = segment[i];

  mavlink_mission_item_t *mi_prev = &this->segment[0];
  mavlink_mission_item_t *mi      = &this->segment[1];
  sphere.updatePoints(mi_prev->x, mi_prev->y, mi->x, mi->y);
}

/**
 *
 */
void Navigator::start(void) {
  ready = true;
}

/**
 *
 */
void Navigator::stop(void) {
  ready = false;
}

/**
 *
 */
void Navigator::start_loiter(void) {
  return;
}


} /* namespace */
