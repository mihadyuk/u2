#include "main.h"
#include "engine.hpp"
#include "float2pwm.hpp"
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
Engine::Engine(PWM &pwm) :
pwm(pwm),
state(EngineState::uninit)
{
  return;
}

/**
 *
 */
void Engine::start(void) {

  param_registry.valueSearch("SRV_thr_min", &thr_min);
  param_registry.valueSearch("SRV_thr_mid", &thr_mid);
  param_registry.valueSearch("SRV_thr_max", &thr_max);

  state = EngineState::disarmed;
}

/**
 *
 */
void Engine::stop(void) {
  state = EngineState::uninit;
}

/**
 *
 */
void Engine::update(const DrivetrainImpact &impact) {
  int16_t thrust;

  osalDbgCheck(EngineState::uninit != state);

  thrust = float2pwm(impact.ch[IMPACT_THR], *thr_min, *thr_mid, *thr_max);

  if (EngineState::armed == state)
    pwm.update(thrust, PWM_CH_THR);
  else if (EngineState::disarmed == state)
    pwm.update(THRUST_DISARMED_VALUE, PWM_CH_THR);
  else
    osalSysHalt("Unhandled value");
}

/**
 *
 */
void Engine::arm(void) {
  state = EngineState::armed;
}

/**
 *
 */
void Engine::disarm(void) {
  state = EngineState::disarmed;
}

