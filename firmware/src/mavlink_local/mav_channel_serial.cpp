#include "main.h"
#include "mav_channel_serial.hpp"

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

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
mavChannelSerial::mavChannelSerial() :
sdp(nullptr)
{
  return;
}

/**
 *
 */
void mavChannelSerial::start(SerialDriver *sdp){
  chDbgCheck((NULL != sdp) && (SD_READY == sdp->state));
  this->sdp = sdp;
  this->ready = true;
}

/**
 *
 */
void mavChannelSerial::stop(void){
  if (ready){
    this->sdp = nullptr;
    this->ready = false;
  }
}

/**
 *
 */
msg_t mavChannelSerial::write(const uint8_t *buf, size_t len, systime_t timeout) {
  osalDbgCheck(ready);
  return chnWriteTimeout(sdp, buf, len, timeout);
}

/**
 *
 */
msg_t mavChannelSerial::get(systime_t time) {
  osalDbgCheck(ready);
  return chnGetTimeout(sdp, time);
}

/**
 *
 */
size_t mavChannelSerial::read(uint8_t *buf, size_t len, systime_t timeout){
  osalDbgCheck(ready);
  return chnReadTimeout(sdp, buf, len, timeout);
}
