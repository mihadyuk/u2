#ifndef MESSAGE_H_
#define MESSAGE_H_

#ifndef MAVLINK_CHECK_MESSAGE_LENGTH
#define MAVLINK_CHECK_MESSAGE_LENGTH    TRUE
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include "mavlink.h"
#pragma GCC diagnostic pop

/* input messages events */
#define EVMSK_MAVLINK_IN_PARAM_SET                          (1UL << 0)
#define EVMSK_MAVLINK_IN_PARAM_REQUEST_LIST                 (1UL << 1)
#define EVMSK_MAVLINK_IN_PARAM_REQUEST_READ                 (1UL << 2)
#define EVMSK_MAVLINK_IN_COMMAND_LONG                       (1UL << 3)
#define EVMSK_MAVLINK_IN_MANUAL_CONTROL                     (1UL << 4)
#define EVMSK_MAVLINK_IN_SET_MODE                           (1UL << 5)
#define EVMSK_MAVLINK_IN_MISSION_SET_CURRENT                (1UL << 6)
#define EVMSK_MAVLINK_IN_MISSION_REQUEST_LIST               (1UL << 7)
#define EVMSK_MAVLINK_IN_MISSION_COUNT                      (1UL << 8)
#define EVMSK_MAVLINK_IN_MISSION_CLEAR_ALL                  (1UL << 9)
#define EVMSK_MAVLINK_IN_MISSION_ITEM                       (1UL << 10)
#define EVMSK_MAVLINK_IN_MISSION_REQUEST                    (1UL << 11)
#define EVMSK_MAVLINK_IN_MISSION_ACK                        (1UL << 12)
#define EVMSK_MAVLINK_IN_HEARTBEAT                          (1UL << 13)
#define EVMSK_MAVLINK_IN_GRIFFON_MEASUREMENT                (1UL << 14)
#define EVMSK_MAVLINK_IN_GRIFFON_BINLOG_REQUEST_SIZE        (1UL << 15)
#define EVMSK_MAVLINK_IN_GRIFFON_BINLOG_REQUEST_BLOCK_INFO  (1UL << 16)
#define EVMSK_MAVLINK_IN_GRIFFON_BINLOG_REQUEST_DATA        (1UL << 17)
#define EVMSK_MAVLINK_IN_GRIFFON_BINLOG_RELEASE             (1UL << 18)
#define EVMSK_MAVLINK_IN_GRIFFON_BINLOG_ABORT               (1UL << 19)
#define EVMSK_MAVLINK_IN_GRIFFON_BINLOG_ERASE               (1UL << 20)
#define EVMSK_MAVLINK_IN_GRIFFON_VM_TRANSIT                 (1UL << 21)

/* other events */
#define EVMSK_MISSION_UPDATED                               (1UL << 31)

/* value denoting end of transit filtering array */
#define TRANSIT_DROP_END                                    255

/* function prototypes */
void MsgInit(void);

#endif /* MESSAGE_H_ */

