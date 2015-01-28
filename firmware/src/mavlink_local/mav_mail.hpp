#ifndef MAVMAIL_HPP_
#define MAVMAIL_HPP_

#include "main.h"
#include "mavlink_local.hpp"

/**
 *
 */
class mavMail {
public:
  mavMail(void);
  void fill(const void *mavmsg, MAV_COMPONENT compid, uint8_t msgid);
  virtual void release(void);
  bool free(void);

  const void *mavmsg;
  MAV_COMPONENT compid;
  uint8_t msgid;
};

/**
 * @brief     Mavlink mail class with synchronization mechanism
 */
class mavMailSync : public mavMail, public chibios_rt::BinarySemaphore {
public:
  mavMailSync(void) : chibios_rt::BinarySemaphore(true) {
    return;
  }
  void release(void);
};

#endif /* MAVMAIL_HPP_ */
