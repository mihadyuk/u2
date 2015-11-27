#ifndef MAV_POSTMAN_HPP_
#define MAV_POSTMAN_HPP_

#include "main.h"
#include "mav_channel.hpp"
#include "mav_mail.hpp"
#include "mav_spam_list.hpp"

#define WORKER_RX_THREAD_WA_SIZE  256
#define WORKER_TX_THREAD_WA_SIZE  512

class MavPostman {
public:
  MavPostman(void);
  void start(mavChannel *channel);
  void stop(void);
  msg_t post(mavMail &mail);
  msg_t postAhead(mavMail &mail);
  void free(mavlink_message_t *msg);
  void subscribe(uint8_t msg_id, SubscribeLink *sl);
  void unsubscribe(uint8_t msg_id, SubscribeLink *sl);
  static MavSpamList spam_list;
private:
  mavChannel *channel = nullptr;
  thread_t *rxworker = nullptr;
  thread_t *txworker = nullptr;
  bool ready = false;
};

extern MavPostman mav_postman;

#endif /* MAV_POSTMAN_HPP_ */
