#pragma GCC optimize "-O0"

#include "main.h"
#include "pads.h"
#include "speedometer.hpp"
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

void speedometer_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p) {
  (void)eicup;
  (void)channel;
  (void)w;

  mavlink_out_debug_vect_struct.y = p;
}

static const EICUChannelConfig speedometercfg = {
    EICU_INPUT_ACTIVE_LOW,
    EICU_INPUT_EDGE,
    speedometer_cb
};

/* for timer 9 */
static const EICUConfig eicucfg = {
    (1000 * 50), /* EICU clock frequency (about 50kHz for meaningful results).*/
    {
        &speedometercfg,
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
void Speedometer::start(void) {

  eicuStart(&EICUD11, &eicucfg);
  eicuEnable(&EICUD11);
}

/**
 *
 */
void Speedometer::stop(void) {

  eicuDisable(&EICUD11);
  eicuStop(&EICUD11);
}
