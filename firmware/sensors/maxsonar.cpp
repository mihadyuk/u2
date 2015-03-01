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
  //mavlink_out_debug_vect_struct.x = eicuGetWidth(eicup, channel) * 0.0001724137;
  mavlink_out_debug_vect_struct.x = eicuGetTime(eicup, channel);
  red_led_toggle();
}

void overflow_cb(EICUDriver *eicup, eicuchannel_t channel) {
  (void)eicup;
  (void)channel;

  //blue_led_toggle();
}

static EICUChannelConfig sonarcfg = {
    EICU_INPUT_ACTIVE_HIGH,
    sonar_cb
};

/* for timer 9 */
static EICUConfig eicucfg = {
    EICU_INPUT_PULSE,//EICU_INPUT_EDGE,//,
    (1000 * 1000),      /* EICU clock frequency (Hz).*/
    {
        &sonarcfg,
        NULL,
        NULL,
        NULL
    },
    NULL,
    NULL,//overflow_cb,
    0
};

///* for timers 2 or 5 */
//static EICUConfig eicucfg = {
//    EICU_INPUT_PULSE,
//    (1000 * 1000),      /* EICU clock frequency (Hz).*/
//    {
//        NULL,
//        NULL,
//        &sonarcfg,
//        NULL
//    },
//    NULL,
//    NULL,//overflow_cb,
//    0
//};

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


