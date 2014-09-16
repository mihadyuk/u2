#ifndef MAV_POSTMAN_HPP_
#define MAV_POSTMAN_HPP_

#include "subscribe_link.hpp"

/**
 *
 */
class MavPostman {
public:
  MavPostman(void);
  void subscribe(uint8_t msg_id, SubscribeLink *sl);
  void unsubscribe(uint8_t msg_id, SubscribeLink *sl);
  void dispatch(const mavlink_message_t &msg);
private:
  int search(uint8_t msg_id);
};

extern MavPostman mav_postman;

#endif /* MAV_POSTMAN_HPP_ */
