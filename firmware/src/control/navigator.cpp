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
void Navigator::update(const NavIn<float> &in, NavOut<float> &out) {

  osalDbgCheck(ready);

  /* get cross track error */
  auto crosstrack = sphere.crosstrack(in.lat, in.lon);
  out.xtd = crosstrack.xtd;
  out.atd = crosstrack.atd;

  /* get target distance and cource */
  auto crs_dist = sphere.course_distance(in.lat, in.lon);
  out.crs = crs_dist.crs;
  out.dist = crs_dist.dist;
}

/**
 *
 */
void Navigator::loadLine(const NavLine<float> &line) {

  sphere.updatePoints(line.latA, line.lonA, line.latB, line.lonB);
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

} /* namespace */
