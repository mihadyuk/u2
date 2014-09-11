#ifndef MAVCHANNEL_USBSERIAL_H_
#define MAVCHANNEL_USBSERIAL_H_

#include "main.h"
#include "mavchannel.hpp"

/**
 *
 */
class mavChannelUsbSerial : public mavChannel {
public:
  mavChannelUsbSerial(SerialUSBDriver *sdp, const SerialUSBConfig *ser_cfg);
  void start(void);
  void stop(void);
  void write(const uint8_t *buf, size_t len);
  msg_t get(systime_t time);
protected:
  SerialUSBDriver *sdp;
  const SerialUSBConfig *ser_cfg;
};

#endif /* MAVCHANNEL_USBSERIAL_H_ */
