#ifndef SUBSCRIBE_LINK_HPP_
#define SUBSCRIBE_LINK_HPP_

#include "ch.hpp"
#include "message.hpp"


typedef void (*linkcb_t)(mavlink_message_t *msg);


class SubscribeLink {
  friend class MavRegistry;
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





//class SubscribeLink_old {
//public:
//  SubscribeLink_old(chibios_rt::Mailbox *mb) :
//  mailbox(mb),
//  next(NULL)
//  {
//    osalDbgCheck(NULL != mb);
//  };
//  chibios_rt::Mailbox *mailbox;
//  SubscribeLink_old *next;
//};
//
///**
// *
// */
//class SubscribeLink : public chibios_rt::Mailbox {
//  friend class MavRegistry;
//public:
//  SubscribeLink(msg_t *buf, cnt_t n) : Mailbox(buf, n), next(nullptr){;}
//  virtual ~SubscribeLink(void) = 0; /* to make this class fully virtual */
//private:
//  SubscribeLink *next;
//};
//
///**
// *
// */
//template <int N>
//class SubscribeLinkBuf : public SubscribeLink {
//public:
//  SubscribeLinkBuf(void) : SubscribeLink(mb_buf, (cnt_t)(sizeof mb_buf / sizeof (msg_t))){;}
//private:
//  msg_t mb_buf[N];
//};

#endif /* SUBSCRIBE_LINK_HPP_ */
