#ifndef MAV_MAIL_HPP_
#define MAV_MAIL_HPP_

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

  const void *mavmsg = nullptr;
  MAV_COMPONENT compid = MAV_COMP_ID_ALL; /* _sending_ component id */
  uint8_t msgid = 0;
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

#endif /* MAV_MAIL_HPP_ */
