#ifndef MAV_POSTMAN_HPP_
#define MAV_POSTMAN_HPP_

#include "main.h"
#include "mav_channel.hpp"
#include "subscribe_link.hpp"
#include "mav_mail.hpp"
#include "mav_spam_list.hpp"

#if !MAVLINK_UNHANDLED_MSG_DEBUG
#define WORKER_RX_THREAD_WA_SIZE  512
#else
#define WORKER_RX_THREAD_WA_SIZE  2048
#endif

#define WORKER_TX_THREAD_WA_SIZE  1024

#define MAVCHANNEL_BUF_SIZE     MAVLINK_SENDBUF_SIZE
#define MAVCHANNEL_BUF_CNT      3

#if (MAVCHANNEL_BUF_CNT < 2)
#error "Buffer count can not be less than 2"
#endif

class MavPostman {
public:
  MavPostman(void);
  void start(mavChannel *channel);
  void stop(void);
  msg_t post(mavMail &mail);
  void subscribe(uint8_t msg_id, SubscribeLink *sl);
  void unsubscribe(uint8_t msg_id, SubscribeLink *sl);
  static MavSpamList spam_list;
private:
  mavChannel *channel = NULL;
  thread_t *rxworker = NULL;
  thread_t *txworker = NULL;
  bool ready = false;
};

extern MavPostman mav_postman;

#endif /* MAV_POSTMAN_HPP_ */
