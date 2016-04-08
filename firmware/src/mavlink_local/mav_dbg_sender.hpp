#ifndef MAV_DBG_SENDER_HPP_
#define MAV_DBG_SENDER_HPP_

#include "debug_indices.h"

/*
 *
 */
class MavDbgSender {
public:
  void start(void);
  void stop(void);
  void send(float value, uint8_t idx, uint32_t time_boot_ms);
  void send(const char *name, float x, float y, float z, uint64_t time_usec);
private:
  bool ready = false;
};

extern MavDbgSender mav_dbg_sender;

#endif /* MAV_DBG_SENDER_HPP_ */
