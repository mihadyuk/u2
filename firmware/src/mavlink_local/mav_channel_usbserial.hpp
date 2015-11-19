#ifndef MAV_CHANNEL_USBSERIAL_H_
#define MAV_CHANNEL_USBSERIAL_H_

#include "main.h"
#include "mav_channel.hpp"

/**
 *
 */
class mavChannelUsbSerial : public mavChannel {
public:
  mavChannelUsbSerial(void);
  void start(SerialUSBDriver *sdp);
  void stop(void);
  msg_t write(const uint8_t *buf, size_t len, systime_t timeout);
  msg_t get(systime_t time);
  size_t read(uint8_t *buf, size_t len, systime_t timeout);
protected:
  SerialUSBDriver *sdp;
};

#endif /* MAV_CHANNEL_USBSERIAL_H_ */
