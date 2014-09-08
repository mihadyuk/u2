#ifndef MAV_SPAMMER_HPP_
#define MAV_SPAMMER_HPP_

#include "subscribe_link.hpp"

struct mav_registry_item_t {
  SubscribeLink *link;
  uint16_t drop_cnt;
  uint8_t msg_id;
};

/**
 *
 */
class MavSpammer {
public:
  MavSpammer(void);
  void add_link(uint8_t msg_id, SubscribeLink *sl);
  void del_link(uint8_t msg_id, SubscribeLink *sl);
  void dispatch(mavlink_message_t *msg);
private:
  int search(uint8_t msg_id);
};

extern MavSpammer mav_spammer;

#endif /* MAV_SPAMMER_HPP_ */
