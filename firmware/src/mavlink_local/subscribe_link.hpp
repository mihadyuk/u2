#ifndef SUBSCRIBE_LINK_HPP_
#define SUBSCRIBE_LINK_HPP_

#include "message.hpp"

typedef void (&linkcb_t)(const mavlink_message_t &msg);

class SubscribeLink {
  friend class MavSpammer;
public:
  SubscribeLink(linkcb_t f) :
  callback(f),
  next(nullptr),
  connected(false)
  {
    osalDbgCheck(nullptr != f);
  };
private:
  linkcb_t callback;
  SubscribeLink *next;
  bool connected;
};

#endif /* SUBSCRIBE_LINK_HPP_ */
