#!/usr/bin/env python

import argparse

parser = argparse.ArgumentParser(description='Description will be here.')

parser.add_argument('-f', '--filename', metavar='F', type=str, required=True,
                   help='file to store results')

args = parser.parse_args()

names_uav = [
    # keep this list alphabetically sorted
    ["attitude",    "attitude"],
    ["global_pos",  "global_position_int"],
    ["gps_raw_int", "gps_raw_int"],
    ["gps_status",  "gps_status"],
    ["highres_imu", "highres_imu"],
    ["mission_curr","mission_current"],
    ["nav_output",  "nav_controller_output"],
    ["position_ned","local_position_ned"],
    # ["raw_imu",     "raw_imu"],
    # ["raw_press",   "raw_pressure"],
    ["rc",          "rc_channels"],
    ["rc_scaled",   "rc_channels_scaled"],
    ["scal_imu",    "scaled_imu"],
    # ["scal_press",  "scaled_pressure"],
    ["sys_status",  "sys_status"],
    ["system_time", "system_time"],
    ["vfr_hud",     "vfr_hud"],
]

names = names_uav

head = """
#include "main.h"
#include "mavlink_local.hpp"
#include "param_registry.hpp"
#include "global_flags.h"
#include "tlm_sender.hpp"
#include "mav_mail.hpp"
#include "mav_postman.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
"""

foot = """
/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
TlmSender::TlmSender(void){
  return;
}

/**
 *
 */
void TlmSender::start(void){

  load_parameters();

  this->worker = chThdCreateStatic(TlmSenderThreadWA,
                 sizeof(TlmSenderThreadWA),
                 TELEMETRYDISPPRIO,
                 TlmSenderThread,
                 NULL);
  osalDbgCheck(NULL != this->worker);

  pause_flag = false;
}

/**
 *
 */
void TlmSender::stop(void){
  pause_flag = true;
  chThdTerminate(worker);
  chThdWait(worker);
}

/**
 *
 */
void TlmSender::pause(void){
  pause_flag = true;
}

/**
 *
 */
void TlmSender::resume(void){
  pause_flag = false;
}
"""

externs_sep = """
/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
"""

prototypes_sep = """
/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */
/* sending function */
typedef void (*send_t)(void);

struct tlm_registry_t {
  tlm_registry_t(systime_t nd, uint32_t const *sp, const send_t s) :
    next_dealine(nd), sleepperiod(sp), sender(s){};
  tlm_registry_t(void) = delete;
  /* how much to sleep */
  systime_t next_dealine;
  /* pointer to period value in global parameters structure */
  uint32_t const *sleepperiod;
  /* sending function */
  const send_t sender;
};

"""

global_vars_sep = """
/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static uint32_t mailbox_overflow = 0;
static uint32_t mail_undelivered = 0;
static bool pause_flag = false;

"""

local_func_sep = """
/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
"""
local_functions = """
/**
 *
 */
static systime_t get_sleep_time(tlm_registry_t *R, size_t len){
  systime_t t;
  uint32_t i;

  t = R[0].next_dealine; /* just take first available */
  i = 0;
  while (i < len){
    /* determine minimum sleep time */
    if (t > R[i].next_dealine)
      t = R[i].next_dealine;
    i++;
  }
  return t;
}

/**
 * refresh deadlines according sleeped time
 */
void refresh_deadlines(tlm_registry_t *R, size_t len, systime_t t){
  uint32_t i = 0;
  while (i < len){
    R[i].next_dealine -= t;
    if (R[i].next_dealine == 0){
      if (*(R[i].sleepperiod) != TELEMETRY_SEND_OFF){
        R[i].next_dealine = *(R[i].sleepperiod);
        R[i].sender();
      }
      else
        R[i].next_dealine = 1200;
    }
    i++;
  }
}
"""

thread_loop = """
/**
 * Listen events with new parameters
 */
static THD_WORKING_AREA(TlmSenderThreadWA, 200) __CCM__;
static THD_FUNCTION(TlmSenderThread, arg) {
  chRegSetThreadName("TLM_Scheduler");
  (void)arg;
  systime_t t; /* milliseconds to sleep to next deadline */

  /* main loop */
  while (!chThdShouldTerminateX()){
    if (true == pause_flag)
      chThdSleepMilliseconds(100);
    else{
      t = get_sleep_time(Registry, sizeof(Registry)/sizeof(Registry[0]));
      chThdSleepMilliseconds(t);
      refresh_deadlines(Registry, sizeof(Registry)/sizeof(Registry[0]), t);
    }
  }

  chThdExit(MSG_OK);
}

"""

def gen(names):
    f = open(args.filename, 'w')
    #f = open("./____tlm_sender.c", 'w')
    f.write("/* \n!!! Automatically generated by\n" + __file__ + "\nDo not edit it manually. \n*/")
    f.write(head)

    # externs
    f.write(externs_sep)
    for i in names:
        f.write("extern const mavlink_" + i[1] + "_t mavlink_out_" + i[1] + "_struct;\n")
    f.write("\n")

    # prototypes
    f.write(prototypes_sep)
    for i in names:
        f.write("static void send_" +i[0]+ "(void);\n")

    # global_vars
    f.write(global_vars_sep)
    for n in names:
        f.write("static mavMail " + n[1] + "_mail __CCM__;\n")
    f.write("\n")

    # parameter registry
    f.write("/* autoinitialized array */\n")
    f.write("__CCM__ static tlm_registry_t Registry[] = {\n")
    n = 10
    for i in names:
        n += 1
        f.write("    {"+str(n)+", nullptr, send_"+i[0]+"},\n")
    f.write("};\n")

    # local functions
    f.write(local_func_sep)
    for i in names:
        f.write("static void send_" +i[0]+ "(void){\n")
        f.write("  msg_t status = MSG_RESET;\n")
        f.write("  if ("+i[1]+"_mail.free()){\n")
        f.write("    "+i[1]+"_mail.fill(&mavlink_out_" + i[1] + "_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_" + str.upper(i[1]) + ");\n")
        f.write("    status = mav_postman.post("+i[1]+"_mail);\n")
        f.write("    if (status != MSG_OK){\n")
        f.write("      mailbox_overflow++;\n")
        f.write("      "+i[1]+"_mail.release();\n")
        f.write("    }\n")
        f.write("  }\n")
        f.write("  else\n")
        f.write("    mail_undelivered++;\n")
        f.write("}\n\n")

    # local functions
    f.write(local_functions)

    # thread
    f.write(thread_loop)

    f.write("/** \n *\n */\n")
    f.write("static void load_parameters(void) {\n")
    n = 0
    for i in names:
        f.write("  param_registry.valueSearch(\"T_" +i[0]+ "\", &(Registry[" + str(n) + "].sleepperiod));\n")
        n += 1
    f.write("}\n")

    # footer
    f.write(foot)
    f.close()


# main function
gen(names)
