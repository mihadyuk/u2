#include <stdio.h>

#include "main.h"

#include "ublox.hpp"
#include "mtkgps.hpp"
#include "cli.hpp"

using namespace chibios_rt;

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
extern memory_heap_t ThdHeap;
extern gnss::GNSSReceiver &GNSS_CLI;

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

void gnss_set_1hz(void) {
  cli_println("Unrealized yet");
}

void gnss_set_5hz(void) {
  cli_println("Unrealized yet");
}

/**
 *
 */
static THD_WORKING_AREA(LoopCmdThreadWA, 1024);
static THD_FUNCTION(LoopCmdThread, arg) {
  chRegSetThreadName("LoopCmd");

  GNSS_CLI.setSniffer((BaseChannel *)arg);

  while (!chThdShouldTerminateX()) {
    chThdSleepMilliseconds(100);
  }

  GNSS_CLI.deleteSniffer();
  chThdExit(MSG_OK);
}

/**
 *
 */
thread_t* loop_sniff(BaseChannel *bchnp) {
  thread_t *loop_clicmd_tp = nullptr;

  loop_clicmd_tp = chThdCreateFromHeap(&ThdHeap,
                                  sizeof(LoopCmdThreadWA),
                                  CMDPRIO - 1,
                                  LoopCmdThread,
                                  bchnp);

  if (loop_clicmd_tp == nullptr)
    osalSysHalt("can not allocate memory");

  return loop_clicmd_tp;
}

/**
 *
 */
static void print_help(void) {
  cli_println("Available commands:");
  cli_println("    sniff");
  cli_println("    1Hz (unimplemented)");
  cli_println("    5Hz (unimplemented)");
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
thread_t* gnss_clicmd(int argc, const char * const * argv, BaseChannel *bchnp) {

  if (0 == argc) {
    cli_println("Not enough arguments");
    goto EXIT;
  }

  if (argc > 0) {
    if (0 == strcmp(argv[0], "sniff")) {
      return loop_sniff(bchnp);
    }
    else if (0 == strcmp(argv[0], "1hz")) {
      gnss_set_1hz();
    }
    else if (0 == strcmp(argv[0], "5hz")) {
      gnss_set_5hz();
    }
    else {
      cli_println("Unknown option");
      print_help();
    }
  }

EXIT:
  return NULL;
}




