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
/**
 * @brief     Routing tables.
 * @details   Maps receiver's PWM channels to abstract channels from
 *            higher levels.
 */
struct route_table_impact_t {
  uint8_t roll    = 0;
  uint8_t pitch   = 1;
  uint8_t yaw     = 2;
  uint8_t speed   = 3;
};

struct route_table_direction_t {
  uint8_t course  = 0;
  uint8_t height  = 1;
  uint8_t speed   = 3;
};

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
static const route_table_impact_t     route_impact;
static const route_table_direction_t  route_direction;

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
void pwm2direction(TargetDirection &dir, const uint16_t *pwm) {

  dir.a[DIRECTION_CH_COURSE] = pwm[0] / 1000.0f;
}

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

/**
 *
 */
void pwm2impact(Impact &impact, const uint16_t *pwm) {

  impact.a[IMPACT_CH_ROLL]  = pwm_normalize(pwm[route_impact.roll]);
  impact.a[IMPACT_CH_PITCH] = pwm_normalize(pwm[route_impact.pitch]);
  impact.a[IMPACT_CH_YAW]   = pwm_normalize(pwm[route_impact.yaw]);
  impact.a[IMPACT_CH_SPEED] = pwm_normalize(pwm[route_impact.speed]);
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
    pwm2direction(result.direction, pwm);
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
    //if manual_switch_RC
    result.level = OverrideLevel::none;
    result.impact.mask = 0;
    result.pwm_vector.mask = 0;
    return ret;
  }

  /* */
  ret = process_pwm(result, receiver_mavlink);
  if (MSG_OK == ret) {
    // TODO: set apropriate override flags here
    //if manual_switch_Mavlink
    result.level = OverrideLevel::none;
    result.impact.mask = 0;
    result.pwm_vector.mask = 0;
    return ret;
  }

  return ret;
}

