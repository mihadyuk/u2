#include "main.h"

#include "futaba/receiver_mavlink.hpp"

using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define RC_MAVLINK_TIMEOUT      MS2ST(250)
#define NORMALIZE_SHIFT         1500
#define NORMALIZE_SCALE         500

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

__CCM__ static RecevierOutput cache;

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
static uint16_t __scale(int16_t val) {
  return val / 2 + 1500;
}

/**
 *
 */
static void mavlink2receiver(const mavlink_manual_control_t &mav, RecevierOutput &recv) {
  recv.ch[0] = __scale(mav.r);
  recv.ch[1] = __scale(mav.z);

  recv.data_valid = true;
  recv.normalize_scale = NORMALIZE_SCALE;
  recv.normalize_shift = NORMALIZE_SHIFT;
  recv.channels = 4;
}

/**
 *
 */
THD_FUNCTION(ReceiverMavlink::RCMavlinkThread, arg) {
  chRegSetThreadName("RC_Mavlink");
  ReceiverMavlink *self = static_cast<ReceiverMavlink *>(arg);
  mavlink_manual_control_t control;

  /* main loop */
  while (!chThdShouldTerminateX()) {
    if (MSG_OK == self->control_mailbox.fetch(&self->recv_msg, RC_MAVLINK_TIMEOUT)) {
      if (MAVLINK_MSG_ID_MANUAL_CONTROL == self->recv_msg->msgid) {
        mavlink_msg_manual_control_decode(self->recv_msg, &control);
        mav_postman.free(self->recv_msg);

        osalSysLock();
        mavlink2receiver(control, cache);
        osalSysUnlock();
      }
    }
  }

  chThdExit(MSG_OK);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
ReceiverMavlink::ReceiverMavlink(void) :
    control_link(&control_mailbox)
{
  return;
}

/**
 *
 */
void ReceiverMavlink::start(void) {

  mav_postman.subscribe(MAVLINK_MSG_ID_MANUAL_CONTROL,  &control_link);

  worker = chThdCreateStatic(RCMavlinkThreadWA, sizeof(RCMavlinkThreadWA),
                             TELEMETRYDISPPRIO, RCMavlinkThread, this);
  osalDbgCheck(nullptr != worker);

  ready = true;
}

/**
 *
 */
void ReceiverMavlink::stop(void) {
  ready = false;

  chThdTerminate(worker);
  chThdWait(worker);
  worker = nullptr;

  mav_postman.unsubscribe(MAVLINK_MSG_ID_MANUAL_CONTROL,  &control_link);
  control_mailbox.reset();
}

/**
 *
 */
void ReceiverMavlink::update(RecevierOutput &result) {

  osalDbgCheck(ready);

  osalSysLock();
  result = cache;
  osalSysUnlock();
}





