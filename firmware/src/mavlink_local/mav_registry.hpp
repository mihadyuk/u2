#ifndef MAV_REGISTRY_HPP_
#define MAV_REGISTRY_HPP_

#include "subscribe_link.hpp"

struct mav_registry_item_t {
  SubscribeLink *link;
  uint8_t msg_id;
};

/**
 *
 */
class MavRegistry {
public:
  MavRegistry(void);
  void add_link(uint8_t msg_id, SubscribeLink *sl);
  void del_link(uint8_t msg_id, SubscribeLink *sl);
  SubscribeLink *get_chain(uint8_t msg_id);
private:
  int search(uint8_t msg_id);
  mav_registry_item_t *registry;
  static const size_t registry_len;
};

#endif /* MAV_REGISTRY_HPP_ */
