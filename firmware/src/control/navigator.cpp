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
  float lat = acs_in.ch[ACS_INPUT_lat];
  float lon = acs_in.ch[ACS_INPUT_lon];

  /* get cross track error */
  auto crosstrack = sphere.crosstrack(lat, lon);
  acs_in.ch[ACS_INPUT_dZ] = crosstrack.xtd;

  /* get target distance and cource */
  auto crc_dist = sphere.course_distance(lat, lon);

  /* make dicision */
  if (crc_dist.dist < 10)
    return NavigatorStatus::navigate;
  else
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
