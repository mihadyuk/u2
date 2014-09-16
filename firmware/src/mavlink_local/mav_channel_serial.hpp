#ifndef MAV_CHANNEL_SERIAL_H_
#define MAV_CHANNEL_SERIAL_H_

#include "main.h"
#include "mav_channel.hpp"

/**
 *
 */
class mavChannelSerial : public mavChannel {
public:
  mavChannelSerial(SerialDriver *sdp, const SerialConfig *ser_cfg);
  void start(void);
  void stop(void);
  void write(const uint8_t *buf, size_t len);
  msg_t get(systime_t time);
protected:
  SerialDriver *sdp;
  const SerialConfig *ser_cfg;
};

#endif /* MAV_CHANNEL_SERIAL_H_ */
