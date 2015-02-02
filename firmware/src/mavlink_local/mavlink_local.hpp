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
#define THIS_COMPONENT_ID           MAV_COMP_ID_SYSTEM_CONTROL

void MavlinkInit(void);

#endif /* MAVLINK_LOCAL_H_ */

