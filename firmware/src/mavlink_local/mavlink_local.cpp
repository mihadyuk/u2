#include "main.h"

#include "mavlink_local.hpp"
#include "param_registry.hpp"

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

/* variable for storing system state */
__CCM__ mavlink_system_t        mavlink_system_struct;
__CCM__ mavlink_system_info_t   mavlink_system_info_struct;

/* mavlink messages containing telemetry data
 * keep them alphabetically sorted for convenience */
mavlink_attitude_t              mavlink_out_attitude_struct;
mavlink_attitude_quaternion_t   mavlink_out_attitude_quaternion_struct;
mavlink_command_ack_t           mavlink_out_command_ack_struct;
mavlink_global_position_int_t   mavlink_out_global_position_int_struct;
mavlink_gps_raw_int_t           mavlink_out_gps_raw_int_struct;
mavlink_gps_status_t            mavlink_out_gps_status_struct;
mavlink_heartbeat_t             mavlink_out_heartbeat_struct;
mavlink_highres_imu_t           mavlink_out_highres_imu_struct;
mavlink_hil_state_t             mavlink_out_hil_state_struct;
mavlink_local_position_ned_t    mavlink_out_local_position_ned_struct;
mavlink_mission_current_t       mavlink_out_mission_current_struct;
mavlink_mission_item_reached_t  mavlink_out_mission_item_reached_struct;
mavlink_nav_controller_output_t mavlink_out_nav_controller_output_struct;
//mavlink_raw_imu_t               mavlink_out_raw_imu_struct;
//mavlink_raw_pressure_t          mavlink_out_raw_pressure_struct;
mavlink_rc_channels_scaled_t    mavlink_out_rc_channels_scaled_struct;
mavlink_rc_channels_t           mavlink_out_rc_channels_struct;
mavlink_scaled_imu_t            mavlink_out_scaled_imu_struct;
//mavlink_scaled_pressure_t       mavlink_out_scaled_pressure_struct;
mavlink_sys_status_t            mavlink_out_sys_status_struct;
mavlink_system_time_t           mavlink_out_system_time_struct;
mavlink_vfr_hud_t               mavlink_out_vfr_hud_struct;

/**
 * @brief   Event sources.
 */
EvtSource event_mission_updated;
EvtSource event_mission_reached;
EvtSource event_parameters_updated;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
void MavlinkInit(void) {
  uint32_t *type = nullptr;
  uint32_t *id = nullptr;

  param_registry.valueSearch("SYS_mavtype", &type);
  param_registry.valueSearch("SYS_id", &id);

  /* mavlink initial values */
  mavlink_system_struct.sysid  = *id;
  mavlink_system_struct.compid = MAV_COMP_ID_ALL;

  mavlink_system_info_struct.state  = MAV_STATE_BOOT;
  mavlink_system_info_struct.mode   = MAV_MODE_PREFLIGHT;
  mavlink_system_info_struct.type   = *type;
}

