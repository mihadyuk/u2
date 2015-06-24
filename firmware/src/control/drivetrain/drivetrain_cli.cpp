#include <stdio.h>
#include <string.h>

#include "main.h"
#include "drivetrain.hpp"
#include "cli.hpp"

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
static const Impact *impact_backup = NULL;
static Impact cli_impact;

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
static void _drv_help(void){
  cli_println("toggle   - switch drivetrain from base impact structure to");
  cli_println("           local one and wise versa.");
  cli_println("thrust F - set trust to F. Value must be in range 0..1");
}

/**
 *
 */
static void _drv_toggle(void){
  cli_println("Unrealized");
}

/**
 *
 */
void _drv_thrust(const char * argv){
  float val;
  const float min = 0;
  const float max = 1;
  int sscanf_status;

  if (NULL == impact_backup){
    cli_println("WARNING! Your action will not take effect until you issue 'toggle' subcommand");
    chThdSleepMilliseconds(50);
  }

  sscanf_status = sscanf(argv, "%f", &val);
  if (sscanf_status != 1)
    cli_println("Parameter inconsistent");
  else if ((val < min) || (val > max))
    cli_println("value out of range (0..1)");
  else
    cli_impact.thrust = val;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 * Working with parameters from CLI.
 */
Thread* drivetrain_clicmd(int argc, const char * const * argv, SerialDriver *sdp){
  (void)sdp;

  /* no arguments */
  if (argc == 0)
    _drv_help();

  /* one argument */
  else if (argc == 1){
    if (strcmp(*argv, "help") == 0)
      _drv_help();
    else if(strcmp(*argv, "toggle") == 0)
      _drv_toggle();
    else
      cli_println("ERROR: not enough arguments. Try 'help'.");
  }

  /* two arguments */
  else if (argc == 2){
    if (strcmp(*argv, "thrust") == 0)
      _drv_thrust(argv[1]);
    else
      cli_println("ERROR: unrecognised command. Try 'help'.");
  }

  /* error handler */
  else
    cli_println("ERROR: too many arguments. Try 'help'.");

  /* stub */
  return NULL;
}
