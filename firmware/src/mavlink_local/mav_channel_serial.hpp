#ifndef MAV_CHANNEL_SERIAL_H_
#define MAV_CHANNEL_SERIAL_H_

#include "main.h"
#include "mav_channel.hpp"

/**
 *
 */
class mavChannelSerial : public mavChannel {
public:
  mavChannelSerial(void);
  void start(SerialDriver *sdp);
  void stop(void);
  msg_t write(const uint8_t *buf, size_t len, systime_t timeout);
  msg_t get(systime_t time);
  size_t read(uint8_t *buf, size_t len, systime_t timeout);
protected:
  SerialDriver *sdp;
};

#endif /* MAV_CHANNEL_SERIAL_H_ */
