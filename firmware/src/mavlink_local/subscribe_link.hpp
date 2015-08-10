#ifndef SUBSCRIBE_LINK_HPP_
#define SUBSCRIBE_LINK_HPP_

#include "mavlink_local.hpp"

/**
 *
 */
class SubscribeLink {
  friend class MavSpamList;
public:
  SubscribeLink(void) = delete;
  SubscribeLink(chibios_rt::MailboxBase<mavlink_message_t*> *mb) : mb(mb) {
    osalDbgCheck(nullptr != mb);
  };
private:
  chibios_rt::MailboxBase<mavlink_message_t*> *mb = nullptr;
  SubscribeLink *next = nullptr;
  bool connected = false;
};

#endif /* SUBSCRIBE_LINK_HPP_ */
