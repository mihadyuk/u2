#!/usr/bin/env python

import argparse

parser = argparse.ArgumentParser(description='Description will be here.')

parser.add_argument('-f', '--filename', metavar='F', type=str, required=True,
                   help='file to store results')

args = parser.parse_args()

names = [
    "attitude",
    "attitude_quaternion",
    "command_ack",
    "debug",
    "debug_vect",
    "global_position_int",
    "gps_raw_int",
    "gps_status",
    "heartbeat",
    "highres_imu",
    "hil_state",
    "mission_ack",
    "mission_count",
    "mission_current",
    "mission_item",
    "mission_item_reached",
    "mission_request",
    "nav_controller_output",
    "local_position_ned",
    "param_value",
    "raw_imu",
    "raw_pressure",
    "rc_channels",
    "rc_channels_scaled",
    "scaled_imu",
    "scaled_pressure",
    "sys_status",
    "statustext",
    "system_time",
    "vfr_hud",
]

head = """
#pragma GCC optimize "-O2"

#include "main.h"
#include "stdlib.h"
#include "string.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include "mavlink.h"
#pragma GCC diagnostic pop

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

char *itoa(int value, char *str, int base); /* warning supressor */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static size_t unhandled_message_cnt = 0;
__CCM__ static mavlink_statustext_t text;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */
/**
 *
 */
static uint16_t error_message(uint8_t msgid, MAV_COMPONENT compid, mavlink_message_t* msg) {

  memset(&text, 0, sizeof(text));
  strcpy(text.text, "Unhandled message type: ");

  char *b = &text.text[strlen(text.text)];
  itoa(msgid, b, 10);
  osalDbgAssert(strlen(text.text) < sizeof(text.text), "overflow");

  text.severity = MAV_SEVERITY_WARNING;

  unhandled_message_cnt++;

  return mavlink_msg_statustext_encode(mavlink_system_struct.sysid, compid, msg, &text);
}

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

  switch(msgid) {
"""

foot = """
  default:
    return error_message(msgid, compid, msg);
  }
}

"""

def gen(names, f):
    f.write("/*\n!!! Automatically generated by\n" + __file__ + "\nDo not edit it manually. \n*/\n")
    f.write(head)
    for i in names:
        f.write("  case MAVLINK_MSG_ID_" + str.upper(i) + ":\n")
        f.write("    return mavlink_msg_" + i + "_encode(mavlink_system_struct.sysid, compid, msg, mavlink_struct);\n")
    f.write(foot)


# main function
f = open(args.filename, 'w')
gen(names, f)
f.close()
