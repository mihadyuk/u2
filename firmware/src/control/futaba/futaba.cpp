#include "main.h"
#include "putinrange.hpp"
#include "array_len.hpp"
#include "geometry.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"
#include "futaba.hpp"

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
float pwm_normalize(uint16_t v, float shift, float scale) {
  return putinrange(((float)v - shift) / scale, -1.0f, 1.0f);
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
    if (recv.data_valid) {
      man = static_cast<ManualSwitch>(manual_switch.update(tmp));
    }
  }
}

/**
 *
 */
static void scale(RecevierOutput const &recv, ACSInput &result) {
  static_assert(ACS_INPUT_futaba_raw_end - ACS_INPUT_futaba_raw_00 ==
      MAX_RC_CHANNELS, "Checker for allowing loop based conversion");

  float *out = &result.ch[ACS_INPUT_futaba_raw_00];

  if (recv.data_valid) {
    for (size_t i=0; i<MAX_RC_CHANNELS; i++) {
      out[i] = pwm_normalize(recv.ch[i], recv.normalize_shift, recv.normalize_scale);
    }
  }
}

/**
 *
 */
void Futaba::recevier2futaba(RecevierOutput const &recv, ACSInput &result) {

  process_man_tumbler(recv, result.futaba_man_switch);

  /* first check errors */
  if (recv.data_valid)
    error_rate(0);
  else
    error_rate(100);

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
  param_registry.valueSearch("RC_map_man",  &map_man);

  receiver_rc.start();
  receiver_mavlink.start();

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
void Futaba::update(ACSInput &result, float dT) {
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

