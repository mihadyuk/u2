#include <stdio.h>

#include "main.h"

#include "acs_input.hpp"
#include "hil.hpp"
#include "cli.hpp"
#include "hil_cli.hpp"
#include "geometry.hpp"

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

/**
 *
 */
void hil_attitude(int argc, const char * const * argv) {
  (void)argc;
  (void)argv;

  cli_println("ATT: unimplemented yet");
}

/**
 *
 */
void hil_disable(void) {
  hil.disableAll();
}

/**
 *
 */
void hil_gnss(int argc, const char * const * argv) {
  double lat, lon, alt;
  int sscanf_status;

  if (0 == argc) {
    cli_println("Error: printing of current values unrealized yet");
  }
  else if (3 == argc) {
    sscanf_status = sscanf(argv[0], "%lf", &lat);
    if ((1 != sscanf_status) || (lat > 90) || (lat < -90)) {
      cli_println("Error: Invalid 1 argument");
      return;
    }

    sscanf_status = sscanf(argv[1], "%lf", &lon);
    if ((1 != sscanf_status) || (lon > 180) || (lon < -180)) {
      cli_println("Error: Invalid 2 argument");
      return;
    }

    sscanf_status = sscanf(argv[2], "%lf", &alt);
    if ((1 != sscanf_status) || (alt < -200) || (alt > 5000)) {
      cli_println("Error: Invalid 3 argument");
      return;
    }

    hil.override(deg2rad(lat), ACS_INPUT_lat);
    hil.override(deg2rad(lon), ACS_INPUT_lon);
    hil.override(alt, ACS_INPUT_alt);
  }
  else {
    cli_println("Error: Incorrect argument number");
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
    cli_println("Not enough arguments.");
    cli_println("Usage: hil gnss 53.9268055 27.5913531 500");
    cli_println("Usage: hil disable");
    goto EXIT;
  }

  if (argc > 0) {
    if (0 == strcmp(argv[0], "gnss")){
      hil_gnss(argc-1, argv+1);
    }
    else if (0 == strcmp(argv[0], "att")) {
      hil_attitude(argc-1, argv+1);
    }
    else if (0 == strcmp(argv[0], "disable")) {
      hil_disable();
    }
    else {
      cli_println("Unknown option");
    }
  }

EXIT:
  return NULL;
}




