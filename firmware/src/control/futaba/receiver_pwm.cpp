#include "main.h"
#include "pads.h"
#include "receiver_pwm.hpp"

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
static receiver_data_t cache;

/**
 *
 */
void futaba_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p) {
  (void)eicup;
  (void)channel;
  (void)p;

  cache.pwm[channel] = w;
}

/**
 *
 */
static EICUChannelConfig futabacfg = {
    EICU_INPUT_ACTIVE_HIGH,
    EICU_INPUT_PULSE,
    futaba_cb
};

/**
 *
 */
static EICUConfig eicucfg = {
    (1000 * 1000),      /* EICU clock frequency (Hz).*/
    {
        &futabacfg,
        &futabacfg,
        &futabacfg,
        &futabacfg,
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
void ReceiverPWM::start(systime_t timeout) {

  this->timeout = timeout;

  eicuStart(&EICUD4, &eicucfg);
  eicuEnable(&EICUD4);

  ready = true;
}

/**
 *
 */
void ReceiverPWM::stop(void) {

  ready = false;
  eicuDisable(&EICUD4);
  eicuStop(&EICUD4);
}

/**
 *
 */
void ReceiverPWM::update(receiver_data_t &result) const {

  osalDbgCheck(ready);

  osalSysLock();
  result = cache;
  osalSysUnlock();
}



