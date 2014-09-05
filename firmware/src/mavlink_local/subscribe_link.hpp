#ifndef SUBSCRIBE_LINK_HPP_
#define SUBSCRIBE_LINK_HPP_

#include "ch.hpp"

class SubscribeLink {
public:
  SubscribeLink(chibios_rt::Mailbox *mb) :
  mailbox(mb),
  next(NULL)
  {
    osalDbgCheck(NULL != mb);
  };
  chibios_rt::Mailbox *mailbox;
  SubscribeLink *next;
};

#endif /* SUBSCRIBE_LINK_HPP_ */
