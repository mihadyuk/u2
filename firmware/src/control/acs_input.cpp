#include "main.h"

#include "mavlink_local.hpp"
#include "acs_input.hpp"
#include "geometry.hpp"

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

extern mavlink_global_position_int_t   mavlink_out_global_position_int_struct;

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
void acs_input2mavlink(const ACSInput &acs_in) {
  double tmp;

  tmp = acs_in.ch[ACS_INPUT_lat];
  mavlink_out_global_position_int_struct.lat = rad2deg(tmp) * DEG_TO_MAVLINK;
  tmp = acs_in.ch[ACS_INPUT_lon];
  mavlink_out_global_position_int_struct.lon = rad2deg(tmp) * DEG_TO_MAVLINK;
  mavlink_out_global_position_int_struct.alt = acs_in.ch[ACS_INPUT_alt] * 1000;
  mavlink_out_global_position_int_struct.hdg = acs_in.ch[ACS_INPUT_yaw];
  mavlink_out_global_position_int_struct.time_boot_ms = TIME_BOOT_MS;
}
