#include <control/futaba/pwm_receiver_rover.hpp>
#include <stdio.h>

#include "main.h"

#include "alpha_beta.hpp"
#include "message.hpp"
#include "utils.hpp"
#include "acs_telemetry.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define CHANNELS_USED               8 /* how many first channels used */

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

/**
 *
 */
bool PWMReceiverRover::update_impl(PwmVector *pwm, systime_t timeout){

  chDbgCheck(NULL != pwm, "PWMPlane::update_impl. Null pointer forbidden");

  /**/
  if (((chTimeNow() - last_success_recv)   > timeout) ||
      ((chTimeNow() - last_success_decode) > timeout)) {
    return CH_FAILED;
  }
  else
    return CH_SUCCESS;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
PWMReceiverRover::PWMReceiverRover(void) {
  last_success_decode = 0;
  last_success_recv = 0;
  ready = false;
}

/**
 *
 */
void PWMReceiverRover::start(void){
  last_success_decode = chTimeNow();
  last_success_recv = chTimeNow();
  ready = true;
}

/**
 *
 */
void PWMReceiverRover::stop(void){
  ready = false;
}
