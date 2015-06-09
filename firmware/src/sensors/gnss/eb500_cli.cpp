#include <stdio.h>

#include "main.h"

#include "eb500.hpp"
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

void eb500_set_1hz(void) {
  ;
}

void eb500_set_5hz(void) {
  ;
}

/**
 *
 */
static THD_WORKING_AREA(LoopCmdThreadWA, 1024);
static THD_FUNCTION(LoopCmdThread, arg) {
  chRegSetThreadName("LoopCmd");

  GPSSetDumpHook((SerialDriver *)arg);

  while (!chThdShouldTerminateX()) {
    chThdSleepMilliseconds(100);
  }

  GPSDeleteDumpHook();
  chThdExit(MSG_OK);
}

/**
 *
 */
thread_t* loop_dump(SerialDriver *sdp) {
  thread_t *loop_clicmd_tp = nullptr;

  loop_clicmd_tp = chThdCreateFromHeap(&ThdHeap,
                                  sizeof(LoopCmdThreadWA),
                                  CMDPRIO - 1,
                                  LoopCmdThread,
                                  sdp);

  if (loop_clicmd_tp == nullptr)
    osalSysHalt("can not allocate memory");

  return loop_clicmd_tp;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
thread_t* eb500_clicmd(int argc, const char * const * argv, SerialDriver *sdp) {
  (void)sdp;

  if (0 == argc) {
    cli_println("Not enough arguments");
    goto EXIT;
  }

  if (argc > 0) {
    if (0 == strcmp(argv[0], "dump")) {
      return loop_dump(sdp);
    }
    else if (0 == strcmp(argv[0], "1hz")) {
      eb500_set_1hz();
    }
    else if (0 == strcmp(argv[0], "5hz")) {
      eb500_set_5hz();
    }
    else {
      cli_println("Unknown option");
    }
  }

EXIT:
  return NULL;
}




