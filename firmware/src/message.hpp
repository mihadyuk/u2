#ifndef MESSAGE_H_
#define MESSAGE_H_

#ifndef MAVLINK_CHECK_MESSAGE_LENGTH
#define MAVLINK_CHECK_MESSAGE_LENGTH    TRUE
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include "mavlink.h"
#pragma GCC diagnostic pop

/* other events */
#define EVMSK_MISSION_UPDATED                               (1UL << 31)

#endif /* MESSAGE_H_ */

