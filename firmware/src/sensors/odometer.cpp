#include "main.h"

#include "odometer.hpp"
#include "mavlink_local.hpp"
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

extern mavlink_vfr_hud_t              mavlink_out_vfr_hud_struct;

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
void Odometer::speed2mavlink(const odometer_data_t &result) {

  //mavlink_out_vfr_hud_struct.groundspeed = result.speed * 100; // *100 for gps speed compare
  //mavlink_out_vfr_hud_struct.groundspeed = result.speed * 50; // for PID tuning
//  mavlink_out_vfr_hud_struct.groundspeed = result.path;
  mavlink_out_vfr_hud_struct.groundspeed = result.speed;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void Odometer::start(void) {

  param_registry.valueSearch("SPD_pulse2m", &pulse2m);
  this->start_impl();
  ready = true;
}

/**
 *
 */
void Odometer::stop(void) {

  ready = false;
  this->stop_impl();
}

/**
 *
 */
void Odometer::update(odometer_data_t &result, float dT) {

  osalDbgCheck(ready);
  this->update_impl(result, dT);
  speed2mavlink(result);
}

