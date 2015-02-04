#include "main.h"
#include "engine.hpp"
#include "putinrange.hpp"

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
int16_t float2pwm(float a) {
  int16_t ret = DRIVETRAIN_PWM_PERIOD * a;

  return putinrange(ret, -DRIVETRAIN_PWM_PERIOD, DRIVETRAIN_PWM_PERIOD);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
Engine::Engine(PWM &pwm) :
pwm(pwm) {
  return;
}

/**
 *
 */
void Engine::start(void) {
  ready = true;
}

/**
 *
 */
void Engine::stop(void) {
  ready = false;
}

/**
 *
 */
void Engine::update(const FutabaData &futaba_data, const Impact &impact) {
  int16_t tmp;

  osalDbgCheck(ready);

  if (OverrideLevel::pwm == futaba_data.level) {
    osalSysHalt("Unrealized");
  }
  else {
    tmp = float2pwm(impact.a[IMPACT_CH_SPEED]);
    if (tmp > 0) {
      pwm.update(tmp, PWM_CH_THRUST_FORTH);
      pwm.update(0,   PWM_CH_THRUST_BACK);
    }
    else {
      pwm.update(0,   PWM_CH_THRUST_FORTH);
      pwm.update(tmp, PWM_CH_THRUST_BACK);
    }
  }
}
