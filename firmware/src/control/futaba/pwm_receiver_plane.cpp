#include <control/futaba/pwm_receiver_plane.hpp>
#include <stdio.h>

#include "main.h"

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
static bool pwm_validate(const PwmVector *pwm){
  size_t i = 0;
  uint8_t flag;

  for (i=0; i<=CHANNELS_USED; i++){
    flag = pwm->v[i] >> 12;
    if (flag != 0){
      return CH_FAILED;
    }
  }
  return CH_SUCCESS;
}

/**
 *
 */
bool PWMReceiverPlane::update_impl(PwmVector *pwm, systime_t timeout){
  (void) timeout;
  PwmVector pwm_tmp;

  chDbgCheck(NULL != pwm, "PWMPlane::update_impl. Null pointer forbidden");

  if (CH_SUCCESS == pwm2rs.recv(&pwm_tmp)){
    pwm2telemetry(pwm_tmp);
    if (CH_SUCCESS == pwm_validate(&pwm_tmp)){
      *pwm = pwm_tmp;
      return CH_SUCCESS;
    }
  }
  return CH_FAILED;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
PWMReceiverPlane::PWMReceiverPlane(void) {
  ready = false;
}

/**
 *
 */
void PWMReceiverPlane::start(void){
  this->pwm2rs.start(PWM2RSPRIO);
  ready = true;
}

/**
 *
 */
void PWMReceiverPlane::stop(void){
  ready = false;
  pwm2rs.stop();
}
