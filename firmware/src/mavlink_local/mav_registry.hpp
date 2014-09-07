#ifndef MAV_REGISTRY_HPP_
#define MAV_REGISTRY_HPP_

#include "subscribe_link.hpp"

struct mav_registry_item_t {
  SubscribeLink *link;
  uint16_t drop_cnt;
  const uint8_t msg_id;
};

/**
 *
 */
class MavRegistry {
public:
  MavRegistry(void);
  void add_link(uint8_t msg_id, SubscribeLink *sl);
  void del_link(uint8_t msg_id, SubscribeLink *sl);
  void dispatch(mavlink_message_t *msg);
private:
  int search(uint8_t msg_id);
  mav_registry_item_t *registry;
  static const size_t registry_len;
};

#endif /* MAV_REGISTRY_HPP_ */
