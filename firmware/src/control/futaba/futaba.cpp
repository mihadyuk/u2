#include "main.h"
#include "putinrange.hpp"
#include "array_len.hpp"
#include "geometry.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"
#include "override_level_enum.hpp"
#include "futaba.hpp"

using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
/**
 *
 */
typedef enum {
  RC_OVERRIDE_NONE,
  RC_OVERRIDE_HIGH,
  RC_OVERRIDE_MEDIUM,
  RC_OVERRIDE_LOW,
  RC_OVERRIDE_ENUM_END
}rc_override_level_t;

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
float pwm_normalize(uint16_t v, float shift, float scale) {
  return putinrange(((float)v - shift) / scale, -1, 1);
}

/**
 *
 */
void Futaba::process_man_tumbler(RecevierOutput const &recv, ManualSwitch &man) {

  /* manual switch will be processed separately because I still have no
   * ideas how to do this elegantly inside ACS. */
  if (-1 == *map_man)
    man = ManualSwitch::fullauto;
  else {
    uint16_t tmp = recv.ch[*map_man];
    if ((tmp & RECEIVER_FLAGS_MASK) == RECEIVER_STATUS_NO_ERRORS) {
      man = static_cast<ManualSwitch>(manual_switch.update(tmp));
    }
  }
}

/**
 *
 */
static bool check_errors(RecevierOutput const &recv) {
  for (size_t i=0; i<MAX_RC_CHANNELS; i++) {
    if ((recv.ch[i] & RECEIVER_FLAGS_MASK) != RECEIVER_STATUS_NO_ERRORS)
      return OSAL_FAILED;
  }

  return OSAL_SUCCESS;
}

/**
 *
 */
static void scale(RecevierOutput const &recv, StateVector &result) {
  static_assert(STATE_VECTOR_futaba_raw_end - STATE_VECTOR_futaba_raw_00 ==
      MAX_RC_CHANNELS, "Checker for allowing loop based conversion");

  float *raw_start = &result.ch[STATE_VECTOR_futaba_raw_00];

  for (size_t i=0; i<MAX_RC_CHANNELS; i++) {
    uint16_t tmp = recv.ch[i];
    if ((tmp & RECEIVER_FLAGS_MASK) != RECEIVER_STATUS_NO_ERRORS) {
      raw_start[i] = pwm_normalize(tmp & RECEIVER_DATA_MASK, recv.normalize_shift, recv.normalize_scale);
    }
  }
}

/**
 *
 */
void Futaba::recevier2futaba(RecevierOutput const &recv, StateVector &result) {

  process_man_tumbler(recv, result.futaba_man);

  /* first check errors */
  if (OSAL_SUCCESS == check_errors(recv))
    error_rate(100);
  else
    error_rate(0);

  result.futaba_good = hyst.check(error_rate.get());
  scale(recv, result);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
Futaba::Futaba(void) {
  return;
}

/**
 *
 */
void Futaba::start(void) {

  param_registry.valueSearch("RC_timeout",  &timeout);
  param_registry.valueSearch("RC_override", &override);
  param_registry.valueSearch("RC_map_man",  &map_man);

  receiver_rc.start(timeout);
  receiver_mavlink.start(timeout);

  ready = true;
}

/**
 *
 */
void Futaba::stop(void){

  ready = false;

  receiver_mavlink.stop();
  receiver_rc.stop();
}

/**
 * @brief   Process all receivers in priorities order (higher to lower)
 */
void Futaba::update(StateVector &result, float dT) {
  (void)dT;
  RecevierOutput recv;

  osalDbgCheck(ready);

  receiver_rc.update(recv);
  recevier2futaba(recv, result);
  if (result.futaba_good == true)
    return;

  receiver_mavlink.update(recv);
  recevier2futaba(recv, result);
  if (result.futaba_good == true)
    return;
}

