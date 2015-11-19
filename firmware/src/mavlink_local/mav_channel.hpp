#ifndef MAV_CHANNEL_H_
#define MAV_CHANNEL_H_

#include "main.h"

/**
 *
 */
class mavChannel {
public:
  mavChannel(void){return;}
  virtual void stop(void) = 0;
  virtual msg_t write(const uint8_t *buf, size_t len, systime_t timeout) = 0;
  virtual msg_t get(systime_t timeout) = 0;
  virtual size_t read(uint8_t *buf, size_t len, systime_t timeout) = 0;
protected:
  bool ready = false;
};

#endif /* MAV_CHANNEL_H_ */
