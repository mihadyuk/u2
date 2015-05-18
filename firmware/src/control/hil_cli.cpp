#include <stdio.h>

#include "main.h"

#include "acs_input.hpp"
#include "hil.hpp"
#include "cli.hpp"
#include "hil_cli.hpp"

using namespace chibios_rt;
using namespace control;

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
extern HIL hil;

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

template <typename T>
T single_parser(const char *arg) {
  T ret;

  int sscanf_status;
  sscanf_status = sscanf(arg, "%f", &ret);
}

/**
 *
 */
void hil_attitude(int argc, const char * const * argv) {
  (void)argc;
  (void)argv;
}

/**
 *
 */
void hil_gnss(int argc, const char * const * argv) {
  float lat, lon, alt;
  int sscanf_status;

  if (3 != argc) {
    cli_println("GNSS: Incorrect argument number");
  }
  else {
    sscanf_status = sscanf(argv[0], "%f", &lat);
    if (1 != sscanf_status) {
      cli_println("Invalid 1 argument");
      return;
    }

    sscanf_status = sscanf(argv[1], "%f", &lon);
    if (1 != sscanf_status) {
      cli_println("Invalid 2 argument");
      return;
    }

    sscanf_status = sscanf(argv[2], "%f", &alt);
    if (1 != sscanf_status) {
      cli_println("Invalid 3 argument");
      return;
    }

    osalSysHalt("convert from deg to rad and perform boundaries checks here");
    hil.override(lat, ACS_INPUT_lat);
    hil.override(lon, ACS_INPUT_lon);
    hil.override(alt, ACS_INPUT_alt);
  }
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
thread_t* hil_clicmd(int argc, const char * const * argv, SerialDriver *sdp) {
  (void)sdp;

  if (0 == argc) {
    cli_println("Not enough arguments");
    goto EXIT;
  }

  if (argc > 0) {
    if (0 == strcmp(argv[0], "gnss")){
      hil_gnss(argc-1, argv+1);
    }
    else if (0 == strcmp(argv[0], "att")) {
      hil_attitude(argc-1, argv+1);
    }
    else {
      cli_println("Unknown option");
    }
  }

EXIT:
  return NULL;
}




