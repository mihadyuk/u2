#include "main.h"
#include "alcoi.hpp"

using namespace chibios_rt;
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

static bool validate_pulse(const AlcoiPulse &pulse) {

  if (pulse.ch >= PID_CHAIN_ENUM_END)
    return OSAL_FAILED;

  /* pulse longer than 1s looks dangerous */
  if (pulse.width > 1)
    return OSAL_FAILED;

  if (pulse.lvl > OverrideLevel::bypass)
    return OSAL_FAILED;

  /* pointless value */
  if (pulse.lvl == OverrideLevel::none)
    return OSAL_FAILED;

  return OSAL_SUCCESS;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void Alcoi::start(void) {
  pulse.lvl       = OverrideLevel::none;
  pulse.ch        = PID_CHAIN_AIL;
  pulse.width     = 0;
  pulse.strength  = 0;

  ready = true;
}

/**
 *
 */
void Alcoi::stop(void) {
  ready = false;
}

/**
 *
 */
void Alcoi::update(StabInput &stab, float dT) {

  osalDbgCheck(ready);

  if (pulse_running) {
    if (this->time_elapsed < pulse.width) {
      stab.ch[pulse.ch].override_target = pulse.strength;
      stab.ch[pulse.ch].override_level  = pulse.lvl;
      time_elapsed += dT;
    }
    else { /* pulse end */
      pulse_running = false;
      time_elapsed = 0;
    }
  }
}

/**
 *
 */
bool Alcoi::loadPulse(const AlcoiPulse &pulse) {

  if (OSAL_SUCCESS == validate_pulse(pulse)) {
    this->pulse = pulse;
    pulse_running = true;
    return OSAL_SUCCESS;
  }
  else {
    pulse_running = false;
    return OSAL_FAILED;
  }
}



