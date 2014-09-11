#ifndef MAVCHANNEL_SERIAL_H_
#define MAVCHANNEL_SERIAL_H_

#include "main.h"
#include "mavchannel.hpp"

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

#endif /* MAVCHANNEL_SERIAL_H_ */
