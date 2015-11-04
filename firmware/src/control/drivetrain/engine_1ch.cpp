#include "main.h"

#include "engine_1ch.hpp"
#include "float2servopwm.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"

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
const int16_t THRUST_DISARMED_VALUE = 1500;

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
Engine1ch::Engine1ch(PWMBase &pwm) : pwm(pwm)
{
  return;
}

/**
 *
 */
void Engine1ch::start_impl(void) {

  param_registry.valueSearch("SRV_thr_min", &thr_min);
  param_registry.valueSearch("SRV_thr_mid", &thr_mid);
  param_registry.valueSearch("SRV_thr_max", &thr_max);
}

/**
 *
 */
void Engine1ch::update_impl(const DrivetrainImpact &impact) {
  int16_t thrust;

  thrust2mavlink(impact.ch[IMPACT_THR]);
  thrust = float2servo_pwm(impact.ch[IMPACT_THR], *thr_min, *thr_mid, *thr_max);

  if (EngineState::armed == state)
    pwm.update(thrust, PWM_CH_THR);
  else if (EngineState::disarmed == state)
    pwm.update(THRUST_DISARMED_VALUE, PWM_CH_THR);
  else
    osalSysHalt("Unhandled value");
}
