#include "main.h"

#include "nmea_proto.hpp"
#include "ubx_proto.hpp"
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

chibios_rt::BinarySemaphore GNSSReceiver::pps_sem(true);
chibios_rt::BinarySemaphore GNSSReceiver::protect_sem(false);

chibios_rt::EvtSource event_gnss;

__CCM__ THD_WORKING_AREA(GNSSReceiver::gnssRxThreadWA, 400);

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
GNSSReceiver::GNSSReceiver(SerialDriver *sdp) : sdp(sdp) {
  return;
}

/**
 *
 */
void GNSSReceiver::stop(void) {
  ready = false;

  chThdTerminate(worker);
  chThdWait(worker);
  worker = nullptr;
}

/**
 *
 */
void GNSSReceiver::setSniffer(SerialDriver *sdp) {
  osalDbgCheck(nullptr != sdp);
  this->sniff_sdp = sdp;
}

/**
 *
 */
void GNSSReceiver::deleteSniffer(void) {

  this->sniff_sdp = nullptr;
}

/**
 *
 */
void GNSSReceiver::subscribe(gnss_data_t* result) {
  osalDbgCheck(nullptr != result);

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
void GNSSReceiver::getCache(gnss_data_t &result) {

  osalSysLock();
  result = this->cache;
  osalSysUnlock();
}

/**
 *
 */
void GNSSReceiver::GNSS_PPS_ISR_I(void) {

  pps_sem.signalI();
}
