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
void msnonmea::subscribe_assistance(void){
  mav_postman.subscribe(MAVLINK_MSG_ID_GNSS_ASSISTANCE,  &gnss_assistance_link);
  return;
}

/**
 *
 */
void msnonmea::release_assistance(void){
  mav_postman.unsubscribe(MAVLINK_MSG_ID_GNSS_ASSISTANCE,  &gnss_assistance_link);
  rtcm_mailbox.reset();
}

/**
 *
 */
void msnonmea::assist(void){

  mavlink_gnss_assistance_t rtcm;

  if (MSG_OK == rtcm_mailbox.fetch(&recv_msg, TIME_IMMEDIATE)) {
    if (MAVLINK_MSG_ID_GNSS_ASSISTANCE == recv_msg->msgid) {
      mavlink_msg_gnss_assistance_decode(recv_msg, &rtcm);
      mav_postman.free(recv_msg);
      sdWriteTimeout(this->sdp, rtcm.data, rtcm.len, TIME_IMMEDIATE);
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
    gnss_assistance_link(&rtcm_mailbox)
{
  return;
}

