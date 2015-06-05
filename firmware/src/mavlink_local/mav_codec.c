#pragma GCC optimize "-O2"

#include "main.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include "mavlink.h"
#pragma GCC diagnostic pop

#include "encode_table.h"
#include "decode_table.h"

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

extern mavlink_system_t mavlink_system_struct;

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
uint16_t mavlink_encode(uint8_t msgid, MAV_COMPONENT compid,
                        mavlink_message_t* msg, const void* mavlink_struct) {

  return mavlink_encode_table[msgid](mavlink_system_struct.sysid,
                                     compid,
                                     msg,
                                     mavlink_struct);
}

/**
 * @brief   Use this function to acquire size of memory block for decoded message
 */
size_t mavlink_decode_memsize(const mavlink_message_t* msg) {
  return mavlink_message_size_table[msg->msgid];
}

/**
 *
 */
void mavlink_decode(const mavlink_message_t* msg, void* mavlink_struct) {
  mavlink_decode_table[msg->msgid](msg, mavlink_struct);
}

