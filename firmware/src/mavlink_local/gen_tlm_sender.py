#!/usr/bin/env python

import argparse

parser = argparse.ArgumentParser(description='Description will be here.')

parser.add_argument('-n', '--name', metavar='N', type=str,required=True,
                   help='name of used values array')
parser.add_argument('-c', '--channel', metavar='C', type=str, required=True,
                   help='channel where telemetry to be send')
parser.add_argument('-f', '--filename', metavar='F', type=str, required=True,
                   help='file to store results')

args = parser.parse_args()


names_acs = [
#   T_name          Mavlink name
    ["attitude",    "attitude"],
    ["gps_int",     "global_position_int"],
    ["highres_imu", "highres_imu"],
    ["hil_state",   "hil_state"],
    ["nav_output",  "nav_controller_output"],
    ["raw_imu",     "raw_imu"],
    ["raw_press",   "raw_pressure"],
    ["rc_raw",      "rc_channels_raw"],
    ["rc_scaled",   "rc_channels_scaled"],
    ["scal_imu",    "scaled_imu"],
    ["scal_press",  "scaled_pressure"],
    ["sys_status",  "sys_status"],
    ["vfr_hud",     "vfr_hud"],
]

names_ns = [
    ["vfr_hud", "vfr_hud"],
]

names_uav = [
    ["raw_imu", "raw_imu"],
    ["scal_imu", "scaled_imu"],
    ["sys_status", "sys_status"],
    ["gps_int", "global_position_int"],
    ["attitude", "attitude"],
    ["vfr_hud", "vfr_hud"],
    ["position_ned", "local_position_ned"],
    ["nav_output", "nav_controller_output"],
    ["raw_press", "raw_pressure"],
    ["scal_press", "scaled_pressure"],
]

# check what we have to use as parameter array
if args.name == "ns":
    names = names_ns
elif args.name == "acs":
    names = names_acs
elif args.name == "uav":
    names = names_uav
else:
    raise ValueError("Wrong name")

head = """
#include "main.h"
#include "message.hpp"
#include "param_registry.hpp"
#include "global_flags.h"
#include "this_comp_id.h"
#include "tlm_sender.hpp"

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
                 TELEMTRYPRIO,
                 TlmSenderThread,
                 NULL);
  chDbgCheck(NULL != this->worker, "can not allocate memory");

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

typedef struct tlm_registry_t {
  /* how much to sleep */
  systime_t next_dealine;
  /* pointer to period value in global parameters structure */
  uint32_t const *sleepperiod;
  /* sending function */
  const send_t sender;
}tlm_registry_t;

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
      if (*(R[i].sleepperiod) != SEND_OFF){
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
static WORKING_AREA(TlmSenderThreadWA, 200);
static msg_t TlmSenderThread(void *arg) {
  chRegSetThreadName("TLM_Scheduler");
  (void)arg;
  systime_t t; /* milliseconds to sleep to next deadline */

  while (!GlobalFlags.messaging_ready)
    chThdSleepMilliseconds(50);

  setGlobalFlag(GlobalFlags.tlm_link_ready);

  /* main loop */
  while (!chThdShouldTerminate()){
    if (true == pause_flag)
      chThdSleepMilliseconds(100);
    else{
      t = get_sleep_time(Registry, sizeof(Registry)/sizeof(Registry[0]));
      chThdSleepMilliseconds(t);
      refresh_deadlines(Registry, sizeof(Registry)/sizeof(Registry[0]), t);
    }
  }

  clearGlobalFlag(GlobalFlags.tlm_link_ready);
  chThdExit(0);
  return 0;
}

"""

def gen(names):
    f = open(args.filename, 'w')
    #f = open("./____tlm_sender.c", 'w')
    f.write("/* \n!!! Automatically generated by\n" + __file__ + "\nDo not edit it manually. \n*/")
    f.write(head)

    # externs
    f.write(externs_sep)
    i = args.channel
    f.write("#include \"" + i + "_channel.hpp\"\n")
    f.write("extern mavChannel" + str.upper(i[0]) + i[1:] + " " + i + "_channel;\n\n")
    for i in names:
        f.write("extern mavlink_" + i[1] + "_t mavlink_out_" + i[1] + "_struct;\n")
    f.write("\n")

    # prototypes
    f.write(prototypes_sep)
    for i in names:
        f.write("static void send_" +i[0]+ "(void);\n")

    # global_vars
    f.write(global_vars_sep)
    for n in names:
        f.write("static mavMail " + n[1] + "_mail(NULL);\n")
    f.write("\n")

    # parameter registry
    f.write("/* autoinitialized array */\n")
    f.write("static tlm_registry_t Registry[] = {\n")
    n = 10
    for i in names:
        n += 1
        f.write("    {"+str(n)+", NULL, send_"+i[0]+"},\n")
    f.write("};\n")

    # local functions
    f.write(local_func_sep)
    for i in names:
        c = args.channel

        f.write("static void send_" +i[0]+ "(void){\n")
        f.write("  msg_t status = RDY_RESET;\n")
        f.write("  if ("+i[1]+"_mail.free()){\n")
        f.write("    "+i[1]+"_mail.fill(&mavlink_out_" + i[1] + "_struct, COMP_ID, MAVLINK_MSG_ID_" + str.upper(i[1]) + ");\n")
        f.write("    status = " + c + "_channel.post(&"+i[1]+"_mail, TIME_IMMEDIATE);\n")
        f.write("    if (status != RDY_OK)\n")
        f.write("      mailbox_overflow++;\n")
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
