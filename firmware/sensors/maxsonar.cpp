#pragma GCC optimize "-O0"

#include "main.h"
#include "pads.h"
#include "maxsonar.hpp"
#include "mavlink_local.hpp"

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
extern mavlink_debug_vect_t  mavlink_out_debug_vect_struct;

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

void sonar_cb(EICUDriver *eicup, eicuchannel_t channel) {
  mavlink_out_debug_vect_struct.x = eicuGetWidth(eicup, channel) * 0.0001724137;
  red_led_off();
}

void overflow_cb(EICUDriver *eicup, eicuchannel_t channel) {
  (void)eicup;
  (void)channel;
  if (EICU_CHANNEL_3 == channel) {
    red_led_on();
  }
}

static EICU_IC_Settings sonarcfg = {
    EICU_INPUT_ACTIVE_HIGH,
    sonar_cb
};

static EICUConfig eicucfg = {
    EICU_INPUT_PULSE,
    (1000 * 1000),      /* EICU clock frequency.   */
    {
        NULL,
        NULL,
        &sonarcfg,
        NULL
    },
    NULL,
    overflow_cb,
    0
};

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
void MaxSonar::start(void) {
  eicuStart(&EICUD5, &eicucfg);
  eicuEnable(&EICUD5);
}


