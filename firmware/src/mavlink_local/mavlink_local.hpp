#ifndef MAVLINK_LOCAL_H_
#define MAVLINK_LOCAL_H_

#ifndef MAVLINK_CHECK_MESSAGE_LENGTH
#define MAVLINK_CHECK_MESSAGE_LENGTH    TRUE
#endif

#include "mavlink.h"

#define EVMSK_PARAMETERS_UPDATED    (1UL << 0)
#define EVMSK_MISSION_UPDATED       (1UL << 1)

#define GROUND_SYSTEM_ID            255
/* this realization uses one global component ID for all */
#define GLOBAL_COMPONENT_ID         MAV_COMP_ID_SYSTEM_CONTROL

/**
 * Global structure used to store system's state.
 * In earlier versions of mavlink this iformation was stored in "mavlink_system_t"
 */
typedef struct __mavlink_system_info {
    uint8_t  type;      ///< Stores the system's type (see MAV_TYPE enum)
    uint8_t  state;     ///< Stores the system's state (see MAV_STATE enum)
    uint8_t  mode;      ///< Stores the system's mode (see MAV_MODE_FLAG and MAV_MODE enums)
    uint32_t nav_mode;  ///< Unused, can be used by user to store the system's navigation mode
} mavlink_system_info_t;

/**
 *
 */
extern mavlink_system_t       mavlink_system_struct;

/**
 * Decide if this packed addressed to us.
 */
template <typename T>
static bool mavlink_msg_for_me(T *msg){
  if (msg->target_system != mavlink_system_struct.sysid)
    return false;
  if (mavlink_system_struct.compid == msg->target_component)
    return true;
  else if (MAV_COMP_ID_ALL == msg->target_component)
    return true;
  else
    return false;
}

void MavlinkInit(void);

#endif /* MAVLINK_LOCAL_H_ */

