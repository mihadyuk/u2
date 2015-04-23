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
msg_t Futaba::man_switch_interpret(RecevierOutput const &recv,
                                   FutabaOutput &result) {
  msg_t ret = MSG_OK;

  static_assert(sizeof(recv.ch) == sizeof(result.ch), "checker for temporal code");
  memcpy(result.ch, recv.ch, sizeof(recv.ch));
  result.man = recv.man;

  return ret;
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
msg_t Futaba::update(FutabaOutput &result, float dT) {
  (void)dT;
  RecevierOutput recv;

  osalDbgCheck(ready);


  /* manual switch will be processed separately because I still have no
     * ideas how to do this elegantly inside ACS. */
    if (-1 == *map_man)
      result.man = ManualSwitch::fullauto;
    else
      get_tumbler(*map_man, &result.man, &result.status);

  receiver_rc.update(recv);
  if (RECEIVER_STATUS_NO_ERRORS == recv.status)
    return man_switch_interpret(recv, result);

  receiver_mavlink.update(recv);
  if (RECEIVER_STATUS_NO_ERRORS == recv.status)
    return man_switch_interpret(recv, result);

  return MSG_TIMEOUT;
}

