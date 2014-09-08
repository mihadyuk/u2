#ifndef MAVWORKER_HPP_
#define MAVWORKER_HPP_

#include "main.h"
#include "mavchannel.hpp"
#include "mavspammer.hpp"
#include "multi_buffer.hpp"

#if !MAVLINK_UNHANDLED_MSG_DEBUG
#define WORKER_RX_THREAD_WA_SIZE  256
#else
#define WORKER_RX_THREAD_WA_SIZE  2048
#endif

#define WORKER_TX_THREAD_WA_SIZE  1024

#define MAVCHANNEL_BUF_SIZE     MAVLINK_SENDBUF_SIZE
#define MAVCHANNEL_BUF_CNT      3

#if (MAVCHANNEL_BUF_CNT < 2)
#error "Buffer count can not be less than 2"
#endif

class MavWorker {
public:
  MavWorker(void);
  void start(mavChannel *channel);
  void stop(void);
  void subscribe(uint8_t msg_id, SubscribeLink *sl);
  void unsubscribe(uint8_t msg_id, SubscribeLink *sl);
private:
  mavChannel *channel = NULL;
  thread_t *rxworker = NULL;
  thread_t *txworker = NULL;
  bool ready = false;
  chibios_rt::MailboxBuffer<MAVCHANNEL_BUF_CNT - 1> rxmb; //Box for storing pointer to buffer containing received bytes.
  chibios_rt::MailboxBuffer<12> txmb; //Box for storing IDs of message scheduled to be sent via channel.
  MultiBuffer<uint8_t, MAVCHANNEL_BUF_SIZE, MAVCHANNEL_BUF_CNT> multi_rxbuf;
  /* staticstic counter */
  size_t rxbytes;
  size_t txbytes;
};

#endif /* MAVWORKER_HPP_ */
