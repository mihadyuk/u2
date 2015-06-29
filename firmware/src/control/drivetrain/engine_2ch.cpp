#include <cstdlib>

#include "main.h"

#include "engine_2ch.hpp"
#include "putinrange.hpp"
#include "param_registry.hpp"

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
const int16_t THRUST_DISARMED_VALUE = 0;

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
int16_t Engine2ch::float2pwm(float thr) {
  int16_t pwm = DRIVETRAIN_PWM_PERIOD * putinrange(thr, -1, 1);
  return putinrange(pwm, -DRIVETRAIN_PWM_PERIOD, DRIVETRAIN_PWM_PERIOD);
}

/**
 *
 */
void Engine2ch::write_thrust_pwm(int16_t thrpwm) {

  if (abs(thrpwm) < (int16_t)*thr_dz)
    thrpwm = 0;

  if (thrpwm > 0) {
    pwm.update(0, PWM_CH_THR_REVERSE);
    pwm.update(thrpwm, PWM_CH_THR);
  }
  else {
    pwm.update(abs(thrpwm), PWM_CH_THR_REVERSE);
    pwm.update(0, PWM_CH_THR);
  }
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
Engine2ch::Engine2ch(PWM &pwm) :
pwm(pwm)
{
  return;
}

/**
 *
 */
void Engine2ch::start_impl(void) {
  param_registry.valueSearch("SRV_thr_dz",  &thr_dz);
}

/**
 *
 */
void Engine2ch::update_impl(const DrivetrainImpact &impact) {

  int16_t thrust;

  thrust2mavlink(impact.ch[IMPACT_THR]);
  thrust = this->float2pwm(impact.ch[IMPACT_THR]);

  if (EngineState::armed == state) {
    write_thrust_pwm(thrust);
  }
  else if (EngineState::disarmed == state) {
    pwm.update(THRUST_DISARMED_VALUE, PWM_CH_THR);
    pwm.update(THRUST_DISARMED_VALUE, PWM_CH_THR_REVERSE);
  }
  else
    osalSysHalt("Unhandled value");
}


