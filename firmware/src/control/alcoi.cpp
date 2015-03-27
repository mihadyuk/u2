#include "main.h"
#include "alcoi.hpp"

using namespace chibios_rt;
using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define MAX_PULSE_WIDTH_ACCEPTED    2 // seconds

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
  if (pulse.width > MAX_PULSE_WIDTH_ACCEPTED)
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

  if (pulse_active) {
    if (this->time_elapsed < pulse.width) {
      stab.ch[pulse.ch].override_target = pulse.strength;
      stab.ch[pulse.ch].override_level  = pulse.lvl;
      time_elapsed += dT;
    }
    else { /* pulse end */
      pulse_active = false;
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
    pulse_active = true;
    return OSAL_SUCCESS;
  }
  else {
    pulse_active = false;
    return OSAL_FAILED;
  }
}



