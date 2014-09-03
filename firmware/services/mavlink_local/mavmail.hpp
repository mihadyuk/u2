#ifndef MAVMAIL_HPP_
#define MAVMAIL_HPP_

#include "main.h"
#include "message.hpp"

/**
 *
 */
class mavMail{
public:
  mavMail(void);
  mavMail(chibios_rt::BinarySemaphore *sem);
  /* convenient function */
  void fill(const void *mavmsg, MAV_COMPONENT compid, uint8_t msgid);
  void release(void);
  bool free(void);

public:
  const void *mavmsg;
  MAV_COMPONENT compid;
  uint8_t msgid;

private:
  void constructor_impl(chibios_rt::BinarySemaphore *sem);
  chibios_rt::BinarySemaphore *sem;
};

#endif /* MAVMAIL_HPP_ */
