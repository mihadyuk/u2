#include <control/futaba/futaba.hpp>
#include "main.h"

#include "misc_math.hpp"
#include "servo_numbers.h"
#include "pwm_channel_numbers.h"
#include "acs_telemetry.hpp"
#include "global_flags.h"

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
static const float THRUST_FAILSAFE  = 0.4;
static const float BREAK_FAILSAFE   = 0.9;

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
float Futaba::pwm_normalize(uint16_t v){
  const float shift = 1500;
  const float scale = 500;
  float ret;

  ret = ((float)v - shift) / scale;
  return putinrange(ret, -1, 1);
}

/**
 * @brief   Special normalizer for engine
 */
float Futaba::pwm_normalize_thrust(uint16_t v){
  const float shift = 2000;
  const float scale = 1000;
  float ret;

  ret = (-(float)v + shift) / scale;
  return putinrange(ret, 0, 1);
}

/**
 *
 */
void Futaba::pwm2impact(const PwmVector &pwm, Impact &impact){
  float i;

  i = pwm_normalize(pwm.v[PWM_AIL_CHANNEL]);
  impact.angle[SERVO_NUMBER_LEFT_AIL]    = i;
  impact.angle[SERVO_NUMBER_RIGHT_AIL]   = i;

  i = pwm_normalize(pwm.v[PWM_ELE_CHANNEL]);
  impact.angle[SERVO_NUMBER_LEFT_ELE]    = i;
  impact.angle[SERVO_NUMBER_RIGHT_ELE]   = i;

  i = pwm_normalize(pwm.v[PWM_RUD_CHANNEL]);
  impact.angle[SERVO_NUMBER_LEFT_RUD]    = i;
  impact.angle[SERVO_NUMBER_RIGHT_RUD]   = i;

  i = pwm_normalize(pwm.v[PWM_FLAP_CHANNEL]);
  impact.angle[SERVO_NUMBER_LEFT_FLAP]   = i;
  impact.angle[SERVO_NUMBER_RIGHT_FLAP]  = i;

  i = pwm_normalize(pwm.v[PWM_STRUT_CHANNEL]);
  impact.angle[SERVO_NUMBER_OTHER_STRUT] = i;

  i = pwm_normalize(pwm.v[PWM_BREAK_CHANNEL]);
  impact.angle[SERVO_NUMBER_OTHER_BREAK] = i;

  i = pwm_normalize(pwm.v[PWM_CHUTE_CHANNEL]);
  impact.angle[SERVO_NUMBER_OTHER_CHUTE] = i;

  i = pwm_normalize_thrust(pwm.v[PWM_THRUST_CHANNEL]);
  impact.thrust = i;
}

/**
 *
 */
bool Futaba::process_pwm(manual_switch_t *manual, systime_t pwm_timeout){
  bool pwm_status = CH_FAILED;

  pwm_status = pwm_receiver.update(&pwm_vector, pwm_timeout);

  /**/
  if (NULL != manual){
    /* change manual variable only on successfull receiving */
    if (CH_SUCCESS == pwm_status){
      int man = pwm_vector.v[PWM_SWITCH_MANUAL_CHANNEL];
      *manual = static_cast<manual_switch_t>(manual_switch3.update(man));
    }
  }

  return pwm_status;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
Futaba::Futaba(Impact &impact, PWMReceiver &pwm_receiver) :
impact(impact),
pwm_receiver(pwm_receiver)
{
  ready = false;
}

/**
 *
 */
void Futaba::start(void){
  pwm_receiver.start();
  ready = true;
}

/**
 *
 */
void Futaba::stop(void){
  ready = false;
  pwm_receiver.stop();
}

/**
 *
 */
bool Futaba::update(manual_switch_t *manual, systime_t pwm_timeout){

  bool ret = CH_FAILED;

  chDbgCheck(true == ready, "Futaba not ready");

  ret = process_pwm(manual, pwm_timeout);
  pwm2impact(pwm_vector, impact);
  impact2telemetry(impact);
  return ret;
}

/**
 *
 */
bool Futaba::dryRun(manual_switch_t *manual, systime_t pwm_timeout){

  bool ret = CH_FAILED;

  chDbgCheck(true == ready, "Futaba not ready");

  ret = process_pwm(manual, pwm_timeout);
  return ret;
}

/**
 *
 */
void Futaba::failSafe(void){
  impact.angle[SERVO_NUMBER_LEFT_AIL]    = 0;
  impact.angle[SERVO_NUMBER_RIGHT_AIL]   = 0;
  impact.angle[SERVO_NUMBER_LEFT_ELE]    = 0;
  impact.angle[SERVO_NUMBER_RIGHT_ELE]   = 0;
  impact.angle[SERVO_NUMBER_LEFT_RUD]    = 0;
  impact.angle[SERVO_NUMBER_RIGHT_RUD]   = 0;
  impact.angle[SERVO_NUMBER_LEFT_FLAP]   = 0;
  impact.angle[SERVO_NUMBER_RIGHT_FLAP]  = 0;
  impact.angle[SERVO_NUMBER_OTHER_STRUT] = 0;
  impact.angle[SERVO_NUMBER_OTHER_BREAK] = BREAK_FAILSAFE;
  impact.angle[SERVO_NUMBER_OTHER_CHUTE] = 0;
  impact.thrust = THRUST_FAILSAFE;
}

