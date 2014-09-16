#ifndef MAVLINK_LOCAL_H_
#define MAVLINK_LOCAL_H_

#ifndef MAVLINK_CHECK_MESSAGE_LENGTH
#define MAVLINK_CHECK_MESSAGE_LENGTH    TRUE
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include "mavlink.h"
#pragma GCC diagnostic pop

/* other events */
#define EVMSK_MISSION_UPDATED                               (1UL << 31)

#endif /* MAVLINK_LOCAL_H_ */

