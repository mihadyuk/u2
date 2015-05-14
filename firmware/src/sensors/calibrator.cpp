#include "main.h"

#include "calibrator.hpp"

using namespace chibios_rt;

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
void Calibrator::start(void) {

  state = CalibratorState::idle;
}

/**
 *
 */
void Calibrator::stop(void) {

  state = CalibratorState::uninit;
}

static const systime_t fake_calibration_time = S2ST(6);

/**
 *
 */
CalibratorState Calibrator::update(const ahrs_data_t &data) {
  (void)data;

  osalDbgCheck(CalibratorState::uninit != state);

  if (CalibratorState::idle == state) {
    this->start_time = chVTGetSystemTimeX();
    state = CalibratorState::active;
  }
  else if (CalibratorState::active == state) {
    systime_t end = chVTGetSystemTimeX() + fake_calibration_time;
    if (! chVTIsSystemTimeWithinX(start_time, end)) {
      state = CalibratorState::idle;
    }
  }
  else {
    osalSysHalt("Uninit");
  }

  return this->state;
}




