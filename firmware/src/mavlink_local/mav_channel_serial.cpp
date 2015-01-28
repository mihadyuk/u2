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
void mavChannelSerial::write(const uint8_t *buf, size_t len) {
  osalDbgCheck(ready);
  sdWrite(sdp, buf, len);
}

/**
 *
 */
msg_t mavChannelSerial::get(systime_t time) {
  osalDbgCheck(ready);
  return sdGetTimeout(sdp, time);
}

/**
 *
 */
size_t mavChannelSerial::read(uint8_t *buf, size_t len, systime_t timeout){
  osalDbgCheck(ready);
  return sdReadTimeout(sdp, buf, len, timeout);
}
