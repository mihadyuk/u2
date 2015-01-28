#ifndef SUBSCRIBE_LINK_HPP_
#define SUBSCRIBE_LINK_HPP_

#include "mav_mail.hpp"

class SubscribeLink {
  friend class MavSpamList;
public:
  SubscribeLink(void) = delete;
  SubscribeLink(chibios_rt::MailboxBase<mavMail*> *mb) : mb(mb) {
    osalDbgCheck(nullptr != mb);
  };
private:
  chibios_rt::MailboxBase<mavMail*> *mb = nullptr;
  SubscribeLink *next = nullptr;
  bool connected = false;
};

#endif /* SUBSCRIBE_LINK_HPP_ */
