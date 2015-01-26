#include "main.h"
#include "putinrange.hpp"
#include "array_len.hpp"

#include <control/futaba/futaba.hpp>

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
/**
 *
 */
float pwm_normalize(uint16_t v) {
  const float shift = 1500;
  const float scale = 500;
  float ret;

  ret = ((float)v - shift) / scale;
  return putinrange(ret, -1, 1);
}

///**
// * @brief   Special normalizer for engine
// */
//float Futaba::pwm_normalize_thrust(uint16_t v){
//  const float shift = 2000;
//  const float scale = 1000;
//  float ret;
//
//  ret = (-(float)v + shift) / scale;
//  return putinrange(ret, 0, 1);
//}
//
///**
// *
// */
//void Futaba::pwm2impact(const PwmVector &pwm, Impact &impact){
//  float i;
//
//  i = pwm_normalize(pwm.v[PWM_AIL_CHANNEL]);
//  impact.angle[SERVO_NUMBER_LEFT_AIL]    = i;
//  impact.angle[SERVO_NUMBER_RIGHT_AIL]   = i;
//
//  i = pwm_normalize(pwm.v[PWM_ELE_CHANNEL]);
//  impact.angle[SERVO_NUMBER_LEFT_ELE]    = i;
//  impact.angle[SERVO_NUMBER_RIGHT_ELE]   = i;
//
//  i = pwm_normalize(pwm.v[PWM_RUD_CHANNEL]);
//  impact.angle[SERVO_NUMBER_LEFT_RUD]    = i;
//  impact.angle[SERVO_NUMBER_RIGHT_RUD]   = i;
//
//  i = pwm_normalize(pwm.v[PWM_FLAP_CHANNEL]);
//  impact.angle[SERVO_NUMBER_LEFT_FLAP]   = i;
//  impact.angle[SERVO_NUMBER_RIGHT_FLAP]  = i;
//
//  i = pwm_normalize(pwm.v[PWM_STRUT_CHANNEL]);
//  impact.angle[SERVO_NUMBER_OTHER_STRUT] = i;
//
//  i = pwm_normalize(pwm.v[PWM_BREAK_CHANNEL]);
//  impact.angle[SERVO_NUMBER_OTHER_BREAK] = i;
//
//  i = pwm_normalize(pwm.v[PWM_CHUTE_CHANNEL]);
//  impact.angle[SERVO_NUMBER_OTHER_CHUTE] = i;
//
//  i = pwm_normalize_thrust(pwm.v[PWM_THRUST_CHANNEL]);
//  impact.thrust = i;
//}
//
///**
// *
// */
//bool Futaba::process_pwm(manual_switch_t *manual, systime_t pwm_timeout){
//
//  bool pwm_status = pwm_receiver.update(&pwm_vector, pwm_timeout);
//
//  /**/
//  if (CH_SUCCESS == connection.good(pwm_status, pwm_timeout)){
//    int man = pwm_vector.v[PWM_SWITCH_MANUAL_CHANNEL];
//    *manual = static_cast<manual_switch_t>(manual_switch3.update(man));
//    return CH_SUCCESS;
//  }
//  else
//    return CH_FAILED;
//}

/**
 *
 */
void pwm2impact(Impact &impact, const uint16_t *pwm) {

  for (size_t i=0; i<ArrayLen(impact.a); i++) {
    impact.a[i] = pwm_normalize(pwm[i]);
  }
}

/**
 *
 */
void pwm2drivetrain(PwmVector &drivetrain, const uint16_t *pwm) {

  for (size_t i=0; i<ArrayLen(drivetrain.pwm); i++) {
    drivetrain.pwm[i] = pwm[i];
  }
}

/**
 *
 */
msg_t process_pwm(FutabaData &result, const Receiver &receiver) {
  msg_t ret = MSG_TIMEOUT;
  uint16_t pwm[FUTABA_RECEIVER_PWM_CHANNELS];

  ret = receiver.update(pwm);
  if (MSG_OK == ret) {
    pwm2impact(result.impact, pwm);
    pwm2drivetrain(result.pwm_vector, pwm);
  }

  return ret;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
Futaba::Futaba(systime_t timeout) :
timeout(timeout),
receiver_mavlink(timeout),
receiver_rc(timeout)
{
  return;
}

/**
 *
 */
void Futaba::start(void){
  receiver_mavlink.start();
  receiver_rc.start();
  ready = true;
}

/**
 *
 */
void Futaba::stop(void){
  ready = false;
  receiver_rc.stop();
  receiver_mavlink.stop();
}

/**
 *
 */
msg_t Futaba::update(FutabaData &result) {

  msg_t ret = MSG_TIMEOUT;

  chDbgCheck(ready);

  /* RC has priority over mavlink futaba */
  ret = process_pwm(result, receiver_rc);
  if (MSG_OK == ret) {
    // TODO: set apropriate override flags here
    if manual_switch_RC
    result.override_level = OVERRIDE_LEVEL_NONE;
    result.impact.mask = 0;
    result.pwm_vector.mask = 0;
    return ret;

  }

  /* */
  ret = process_pwm(result, receiver_mavlink);
  if (MSG_OK == ret) {
    // TODO: set apropriate override flags here
    if manual_switch_Mavlink
    result.override_level = OVERRIDE_LEVEL_NONE;
    result.impact.mask = 0;
    result.pwm_vector.mask = 0;
    return ret;
  }

  return ret;
}

