#ifndef MAVLINK_LOCAL_H_
#define MAVLINK_LOCAL_H_

#ifndef MAVLINK_CHECK_MESSAGE_LENGTH
#define MAVLINK_CHECK_MESSAGE_LENGTH    TRUE
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include "mavlink.h"
#pragma GCC diagnostic pop

#define EVMSK_PARAMETERS_UPDATED    (1UL << 0)
#define EVMSK_MISSION_UPDATED       (1UL << 1)

#define GROUND_SYSTEM_ID            255
/* this realization uses one global component ID for all */
#define GLOBAL_COMPONENT_ID         MAV_COMP_ID_SYSTEM_CONTROL

extern mavlink_system_t mavlink_system_struct;

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

