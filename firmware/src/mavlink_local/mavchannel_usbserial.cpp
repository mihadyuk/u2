#include "main.h"
#include "mavchannel_usbserial.hpp"

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
mavChannelUsbSerial::mavChannelUsbSerial(SerialUSBDriver *sdp, const SerialUSBConfig *ser_cfg){
  chDbgCheck((NULL != sdp) &&(NULL != ser_cfg));
  this->sdp = sdp;
  this->ser_cfg = ser_cfg;
}

/**
 *
 */
void mavChannelUsbSerial::start(void){
  sduStart(sdp, ser_cfg);
  this->ready = true;
}

/**
 *
 */
void mavChannelUsbSerial::stop(void){
  if (true == this->ready){
    sduStop(sdp);
    this->ready = false;
  }
}

/**
 *
 */
void mavChannelUsbSerial::write(const uint8_t *buf, size_t len){
  osalDbgCheck(true == this->ready);
  sdWrite(sdp, buf, len);
}

/**
 *
 */
msg_t mavChannelUsbSerial::get(systime_t time){
  osalDbgCheck(true == this->ready);
  return sdGetTimeout(sdp, time);
}


