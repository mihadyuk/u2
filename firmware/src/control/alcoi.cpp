#include "main.h"
#include "alcoi.hpp"
#include "mav_dbg.hpp"

using namespace chibios_rt;
using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define ALCOI_MAX_PULSE_WIDTH     2 // seconds
#define ALCOI_RELAX_TIME          4 // seconds

/* convenience defines */
#define PARAM_PULSE_LEVEL         param4
#define PARAM_PULSE_CHANNEL       param5
#define PARAM_PULSE_WIDTH         param6
#define PARAM_PULSE_STRENGTH      param7

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
  if (pulse.width > ALCOI_MAX_PULSE_WIDTH)
    return OSAL_FAILED;

  if (pulse.lvl > OverrideLevel::bypass)
    return OSAL_FAILED;

  /* pointless value */
  if (pulse.lvl == OverrideLevel::none)
    return OSAL_FAILED;

  return OSAL_SUCCESS;
}

/**
 *
 */
bool Alcoi::load_pulse(const AlcoiPulse &pulse) {

  if (OSAL_SUCCESS == validate_pulse(pulse)) {
    this->pulse = pulse;
    state = AlcoiState::pulse;
    return OSAL_SUCCESS;
  }
  else {
    state = AlcoiState::idle;
    return OSAL_FAILED;
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
void Alcoi::start(void) {
  pulse.lvl       = OverrideLevel::none;
  pulse.ch        = PID_CHAIN_AIL;
  pulse.width     = 0;
  pulse.strength  = 0;

  state = AlcoiState::idle;
}

/**
 *
 */
void Alcoi::stop(void) {
  state = AlcoiState::uninit;
}

/**
 *
 */
void Alcoi::update(StabInput &stab, float dT) {

  osalDbgCheck(AlcoiState::uninit != state);

  switch (state) {
  case AlcoiState::pulse:
    if (this->pulse_time_elapsed < pulse.width) {
      stab.ch[pulse.ch].override_target = pulse.strength;
      stab.ch[pulse.ch].override_level  = pulse.lvl;
      pulse_time_elapsed += dT;
    }
    else { /* pulse end */
      state = AlcoiState::idle;
      pulse_time_elapsed = 0;
    }
    break;

  /**/
  default:
    break;
  }
}

/**
 *
 */
enum MAV_RESULT Alcoi::commandHandler(const mavlink_command_long_t *clp) {
  bool status = OSAL_FAILED;
  AlcoiPulse pulse;

  pulse.lvl     = static_cast<OverrideLevel>(roundf(clp->PARAM_PULSE_LEVEL));
  pulse.ch      = static_cast<pid_chain_t>(roundf(clp->PARAM_PULSE_CHANNEL));
  pulse.width   = clp->PARAM_PULSE_WIDTH;
  pulse.strength= clp->PARAM_PULSE_STRENGTH;

  status = load_pulse(pulse);
  if (OSAL_SUCCESS == status) {
    mavlink_dbg_print(MAV_SEVERITY_INFO, "OK: Alcoi pulse accepted", GLOBAL_COMPONENT_ID);
    return MAV_RESULT_ACCEPTED;
  }
  else {
    return MAV_RESULT_FAILED;
  }
}


