#include "main.h"
#include "putinrange.hpp"
#include "array_len.hpp"
#include "geometry.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"
#include "override_level.hpp"
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
  RC_OVERRIDE_MEDIUM,
  RC_OVERRIDE_LOW,
  RC_OVERRIDE_BYPASS,
  RC_OVERRIDE_PULSE_GENERATOR,
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
static bool is_data_good(receiver_data_t &recv) {

  if ((recv.status & RECEIVER_STATUS_CONN_LOST) != RECEIVER_STATUS_CONN_LOST)
    return true;
  else
    return false;
}

/**
 *
 */
msg_t process_pwm(FutabaData &result, Receiver &receiver) {

  receiver_data_t recv;

  receiver.update(recv);

  return MSG_TIMEOUT;
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

  static_assert(OverrideLevel::none ==
      static_cast<OverrideLevel>(RC_OVERRIDE_NONE),   "");
  static_assert(OverrideLevel::medium ==
      static_cast<OverrideLevel>(RC_OVERRIDE_MEDIUM), "");
  static_assert(OverrideLevel::low ==
      static_cast<OverrideLevel>(RC_OVERRIDE_LOW),    "");
  static_assert(OverrideLevel::bypass ==
      static_cast<OverrideLevel>(RC_OVERRIDE_BYPASS), "");
}

/**
 *
 */
void Futaba::start(void) {

  param_registry.valueSearch("RC_timeout",  &timeout);
  param_registry.valueSearch("RC_override", &override);

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
msg_t Futaba::update(FutabaData &result) {

  msg_t ret = MSG_TIMEOUT;

  osalDbgCheck(ready);

  switch(*override) {
  case RC_OVERRIDE_NONE:
    #warning "TODO:"
    break;
  }

  /* RC */
  ret = process_pwm(result, receiver_rc);
  if (MSG_OK == ret) {
    // TODO: set appropriate override flags here
    //if manual_switch_RC
    return ret;
  }

  /* Mavlink */
  ret = process_pwm(result, receiver_mavlink);
  if (MSG_OK == ret) {
    // TODO: set appropriate override flags here
    //if manual_switch_Mavlink
    return ret;
  }

  return ret;
}

