#ifndef MAV_SPAM_LIST_HPP_
#define MAV_SPAM_LIST_HPP_

#include "subscribe_link.hpp"

/**
 *
 */
class MavSpamList {
public:
  MavSpamList(void);
  void subscribe(uint8_t msg_id, SubscribeLink *sl);
  void unsubscribe(uint8_t msg_id, SubscribeLink *sl);
  void dispatch(const mavlink_message_t &msg);
private:
  int search(uint8_t msg_id);
};

#endif /* MAV_SPAM_LIST_HPP_ */
