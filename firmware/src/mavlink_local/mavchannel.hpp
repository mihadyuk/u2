#ifndef MAVCHANNEL_H_
#define MAVCHANNEL_H_

#include "main.h"

/**
 *
 */
class mavChannel {
public:
  mavChannel(void){return;}
  virtual void start(void) = 0;
  virtual void stop(void) = 0;
  virtual void write(const uint8_t *buf, size_t len) = 0;
  virtual msg_t get(systime_t time) = 0;
protected:
  bool ready = false;
};

#endif /* MAVCHANNEL_H_ */
