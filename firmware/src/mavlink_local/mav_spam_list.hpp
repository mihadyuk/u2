#ifndef MAV_SPAM_LIST_HPP_
#define MAV_SPAM_LIST_HPP_

#include "subscribe_link.hpp"

/**
 *
 */
class MavSpamList {
  friend class MavPostman;
public:
  MavSpamList(void);
  void dispatch(const mavlink_message_t &msg);
private:
  void free(mavlink_message_t *msgptr);
  void subscribe(uint8_t msg_id, SubscribeLink *sl);
  void unsubscribe(uint8_t msg_id, SubscribeLink *sl);
  int search(uint8_t msg_id) const;
  void lock(void) {
    sem.wait();
  }
  void unlock(void) {
    sem.signal();
  }
  chibios_rt::BinarySemaphore sem;
  size_t drop = 0;
};

#endif /* MAV_SPAM_LIST_HPP_ */
