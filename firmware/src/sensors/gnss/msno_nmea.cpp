#include "main.h"

#include "msno_nmea.hpp"

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

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

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
void msnonmea::subscribe_inject(void){
  mav_postman.subscribe(MAVLINK_MSG_ID_GPS_INJECT_DATA,  &inject_link);
  return;
}

/**
 *
 */
void msnonmea::release_inject(void){
  mav_postman.unsubscribe(MAVLINK_MSG_ID_GPS_INJECT_DATA,  &inject_link);
  inject_mailbox.reset();
}

/**
 *
 */
void msnonmea::inject(void){

  mavlink_gps_inject_data_t rtcm;

  if (MSG_OK == inject_mailbox.fetch(&recv_msg, TIME_IMMEDIATE)) {
    if (MAVLINK_MSG_ID_GPS_INJECT_DATA == recv_msg->msgid) {
      mavlink_msg_gps_inject_data_decode(recv_msg, &rtcm);
      mav_postman.free(recv_msg);
      sdWriteTimeout(this->sdp, rtcm.data, rtcm.len, MS2ST(20));
    }
  }
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
msnonmea::msnonmea(SerialDriver *sdp, uint32_t start_baudrate, uint32_t working_baudrate) :
    GenericNMEA(sdp, start_baudrate, working_baudrate),
    inject_link(&inject_mailbox)
{
  return;
}

