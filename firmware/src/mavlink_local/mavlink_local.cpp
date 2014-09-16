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
mavlink_system_t                mavlink_system_struct;

/* mavlink messages */
mavlink_raw_imu_t               mavlink_out_raw_imu_struct;
mavlink_scaled_imu_t            mavlink_out_scaled_imu_struct;
mavlink_raw_pressure_t          mavlink_out_raw_pressure_struct;
mavlink_scaled_pressure_t       mavlink_out_scaled_pressure_struct;
mavlink_sys_status_t            mavlink_out_sys_status_struct;
mavlink_system_time_t           mavlink_out_system_time_struct;
mavlink_vfr_hud_t               mavlink_out_vfr_hud_struct;
mavlink_global_position_int_t   mavlink_out_global_position_int_struct;
mavlink_attitude_t              mavlink_out_attitude_struct;
mavlink_heartbeat_t             mavlink_out_heartbeat_struct;
mavlink_param_value_t           mavlink_out_param_value_struct;
mavlink_gps_raw_int_t           mavlink_out_gps_raw_int_struct;
mavlink_nav_controller_output_t mavlink_out_nav_controller_output_struct;
mavlink_statustext_t            mavlink_out_statustext_struct;
mavlink_command_ack_t           mavlink_out_command_ack_struct;
mavlink_highres_imu_t           mavlink_out_highres_imu_struct;
mavlink_rc_channels_raw_t       mavlink_out_rc_channels_raw_struct;
mavlink_rc_channels_scaled_t    mavlink_out_rc_channels_scaled_struct;
mavlink_hil_state_t             mavlink_out_hil_state_struct;

mavlink_manual_control_t        mavlink_in_manual_control_struct;
mavlink_set_mode_t              mavlink_in_set_mode_struct;
mavlink_param_set_t             mavlink_in_param_set_struct;
mavlink_param_request_read_t    mavlink_in_param_request_read_struct;
mavlink_command_long_t          mavlink_in_command_long_struct;
mavlink_param_request_list_t    mavlink_in_param_request_list_struct;

mavlink_mission_count_t         mavlink_out_mission_count_struct;
mavlink_mission_item_t          mavlink_out_mission_item_struct;
mavlink_mission_request_t       mavlink_out_mission_request_struct;
mavlink_mission_request_list_t  mavlink_out_mission_request_list_struct;
mavlink_mission_ack_t           mavlink_out_mission_ack_struct;
mavlink_mission_clear_all_t     mavlink_out_mission_clear_all_struct;
mavlink_mission_set_current_t   mavlink_out_mission_set_current_struct;
mavlink_mission_current_t       mavlink_out_mission_current_struct;
mavlink_mission_item_reached_t  mavlink_out_mission_item_reached_struct;

mavlink_mission_count_t         mavlink_in_mission_count_struct;
mavlink_mission_item_t          mavlink_in_mission_item_struct;
mavlink_mission_request_t       mavlink_in_mission_request_struct;
mavlink_mission_request_list_t  mavlink_in_mission_request_list_struct;
mavlink_mission_ack_t           mavlink_in_mission_ack_struct;
mavlink_mission_clear_all_t     mavlink_in_mission_clear_all_struct;
mavlink_mission_set_current_t   mavlink_in_mission_set_current_struct;
mavlink_mission_current_t       mavlink_in_mission_current_struct;
mavlink_mission_item_reached_t  mavlink_in_mission_item_reached_struct;

mavlink_heartbeat_t             mavlink_in_heartbeat_struct;

/* lapwing specific packets */

/**
 * @brief   Event sources.
 */
EvtSource event_mission_updated;

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
void MavlinkInit(void){
  /* mavlink initial values */
  mavlink_system_struct.sysid  = 20;
  mavlink_system_struct.compid = MAV_COMP_ID_ALL;
  mavlink_system_struct.state  = MAV_STATE_BOOT;
  mavlink_system_struct.mode   = MAV_MODE_PREFLIGHT;
  mavlink_system_struct.type   = *(uint8_t *)param_registry.getParam("SYS_mavtype", 0, NULL);


}




