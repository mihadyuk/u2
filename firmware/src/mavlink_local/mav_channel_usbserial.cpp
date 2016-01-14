#include "main.h"
#include "mav_channel_usbserial.hpp"

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
mavChannelUsbSerial::mavChannelUsbSerial(void) :
sdp(nullptr)
{
  return;
}

/**
 *
 */
void mavChannelUsbSerial::start(SerialUSBDriver *sdp){
  osalDbgCheck((NULL != sdp) && (SDU_READY == sdp->state));
  this->sdp = sdp;
  this->ready = true;
}

/**
 *
 */
void mavChannelUsbSerial::stop(void){
  if (true == this->ready){
    this->sdp = nullptr;
    this->ready = false;
  }
}

/**
 *
 */
msg_t mavChannelUsbSerial::write(const uint8_t *buf, size_t len, systime_t timeout){
  osalDbgCheck(true == this->ready);
  return chnWriteTimeout(sdp, buf, len, timeout);
}

/**
 *
 */
msg_t mavChannelUsbSerial::get(systime_t timeout){
  osalDbgCheck(true == this->ready);
  return chnGetTimeout(sdp, timeout);
}

/**
 *
 */
size_t mavChannelUsbSerial::read(uint8_t *buf, size_t len, systime_t timeout){
  osalDbgCheck(true == this->ready);
  return chnReadTimeout(sdp, buf, len, timeout);
}
