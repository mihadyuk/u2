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

void sonar_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p) {
  (void)eicup;
  (void)channel;

  //mavlink_out_debug_vect_struct.x = eicuGetWidth(eicup, channel) * 0.0001724137;
  mavlink_out_debug_vect_struct.x = w;
  mavlink_out_debug_vect_struct.y = p;
  red_led_toggle();
}

void overflow_cb(EICUDriver *eicup, eicuchannel_t channel) {
  (void)eicup;
  (void)channel;

  //blue_led_toggle();
}

static EICUChannelConfig sonarcfg = {
    EICU_INPUT_ACTIVE_HIGH,
    EICU_INPUT_BOTH,
    sonar_cb
};

/* for timer 9 */
static EICUConfig eicucfg = {
    EICU_SLOW,
    (100 * 1000),      /* EICU clock frequency (Hz).*/
    {
        &sonarcfg,
        NULL,
        NULL,
        NULL
    },
    NULL,//overflow_cb,
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
  (void)overflow_cb; // warning suppressor

  eicuStart(&EICUD9, &eicucfg);
  eicuEnable(&EICUD9);
}


