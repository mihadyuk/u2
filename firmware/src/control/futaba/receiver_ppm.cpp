#include "main.h"
#include <futaba/receiver_ppm.hpp>

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
void ReceiverPPM::start(systime_t timeout) {
  this->timeout = timeout;
  ready = true;
}

/**
 *
 */
void ReceiverPPM::stop(void) {
  ready = false;
}

/**
 *
 */
void ReceiverPPM::update(receiver_data_t &result) const {

  osalDbgCheck(ready);

  for (size_t i=0; i<RECEIVER_MAX_CHANNELS; i++)
    result.pwm[i] = 1500;
}




