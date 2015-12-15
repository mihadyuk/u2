#include "main.h"

#include "proto_nmea.hpp"
#include "proto_ubx.hpp"
#include "mavlink_local.hpp"
#include "gnss_receiver.hpp"
#include "mav_logger.hpp"
#include "geometry.hpp"
#include "time_keeper.hpp"
#include "pads.h"
#include "chprintf.h"
#include "array_len.hpp"
#include "param_registry.hpp"

using namespace gnss;

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

extern MavLogger mav_logger;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

chibios_rt::BinarySemaphore GNSSReceiver::protect_sem(false);
chibios_rt::EvtSource event_gnss;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
/**
 *
 */
void GNSSReceiver::log_append(const mavlink_gps_raw_int_t *msg) {

  if (gps_raw_int_mail.free()) {
    gps_raw_int_mail.fill(msg, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_GPS_RAW_INT);
    mav_logger.write(&gps_raw_int_mail);
  }
}

/**
 *
 */
void GNSSReceiver::acquire(void) {
  protect_sem.wait();
}

/**
 *
 */
void GNSSReceiver::release(void) {
  protect_sem.signal();
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
GNSSReceiver::GNSSReceiver(SerialDriver *sdp, uint32_t start_baudrate,
                                              uint32_t working_baudrate) :
    sdp(sdp),
    start_baudrate(start_baudrate),
    working_baudrate(working_baudrate)
{
  return;
}

/**
 *
 */
void GNSSReceiver::start(void) {

  osalDbgCheck(! ready);
  osalDbgCheck(nullptr == worker);

  gps_serial_cfg = {0, 0, 0, 0};
  gps_serial_cfg.speed = this->start_baudrate;
  sdStart(this->sdp, &gps_serial_cfg);

  start_impl();
}

/**
 *
 */
void GNSSReceiver::stop(void) {
  ready = false;

  chThdTerminate(worker);
  chThdWait(worker);
  worker = nullptr;

  sdStop(this->sdp);
}

/**
 *
 */
void GNSSReceiver::setSniffer(SerialDriver *sdp) {

  osalDbgCheck(nullptr != sdp);
  osalDbgCheck(ready);

  this->sniff_sdp = sdp;
}

/**
 *
 */
void GNSSReceiver::deleteSniffer(void) {

  osalDbgCheck(ready);

  this->sniff_sdp = nullptr;
}

/**
 *
 */
void GNSSReceiver::subscribe(gnss_data_t* result) {

  osalDbgCheck(nullptr != result);
  osalDbgCheck(ready);

  acquire();

  for (size_t i=0; i<ArrayLen(spamlist); i++) {
    osalDbgAssert(result != spamlist[i],
        "you can not subscribe single structure twice");
  }

  for (size_t i=0; i<ArrayLen(spamlist); i++) {
    if (nullptr == spamlist[i]) {
      spamlist[i] = result;
      release();
      return;
    }
  }

  release();
  osalSysHalt("No free slots remain");
}

/**
 *
 */
void GNSSReceiver::unsubscribe(gnss_data_t* result) {

  osalDbgCheck(nullptr != result);
  osalDbgCheck(ready);

  acquire();

  for (size_t i=0; i<ArrayLen(spamlist); i++) {
    if (result == spamlist[i]) {
      spamlist[i] = nullptr;
      release();
      return;
    }
  }

  release();
  osalSysHalt("This message not subscribed");
}

/**
 *
 */
void GNSSReceiver::GNSS_PPS_ISR_I(void) {
  event_gnss.broadcastFlagsI(EVMSK_GNSS_PPS);
}
