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
//extern mavlink_debug_vect_t  mavlink_out_debug_vect_struct;

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

static void sonar_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p) {
  (void)eicup;
  (void)channel;
  (void)p;

  //mavlink_out_debug_vect_struct.x = eicuGetWidth(eicup, channel) * 0.0001724137;
  (void)w;
}

static const EICUChannelConfig sonarcfg = {
    EICU_INPUT_ACTIVE_HIGH,
    EICU_INPUT_PULSE,
    sonar_cb
};

/* for timer 9 */
static const EICUConfig eicucfg = {
    (1000 * 1000),      /* EICU clock frequency (Hz).*/
    {
        &sonarcfg,
        NULL,
        NULL,
        NULL
    },
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

  eicuStart(&EICUD9, &eicucfg);
  eicuEnable(&EICUD9);
}

/**
 *
 */
void MaxSonar::stop(void) {

  eicuDisable(&EICUD9);
  eicuStop(&EICUD9);
}

