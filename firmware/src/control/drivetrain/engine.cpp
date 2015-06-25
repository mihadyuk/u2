#include "main.h"

#include "engine.hpp"
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
extern mavlink_vfr_hud_t          mavlink_out_vfr_hud_struct;

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
 * @brief   Converts normalized value (-1..1) to mavlink
 */
void Engine::thrust2mavlink(float thr) {

  mavlink_out_vfr_hud_struct.throttle = roundf(putinrange(thr * 100, 0, 100));
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
Engine::Engine(void) :
state(EngineState::uninit)
{
  return;
}

/**
 *
 */
void Engine::start(void) {

  start_impl();
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

  osalDbgCheck(EngineState::uninit != state);
  update_impl(impact);
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

