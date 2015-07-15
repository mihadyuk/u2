#include "main.h"

#include "mav_postman.hpp"
#include "mav_spam_list.hpp"
#include "mav_codec.h"
#include "multi_buffer.hpp"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

MavPostman mav_postman;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
__CCM__ static chibios_rt::Mailbox<mavMail*, 12> txmb;
MavSpamList MavPostman::spam_list;

__CCM__ static mavlink_message_t rx_msg;
__CCM__ static mavlink_status_t rx_status;
__CCM__ static mavlink_message_t tx_msg;
static uint8_t sendbuf[MAVLINK_SENDBUF_SIZE]; /* do not set CCM here. This buffer may be used to send data via DMA */

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */
#include "pads.h"
/**
 *
 */
static THD_WORKING_AREA(RxThreadWA, WORKER_RX_THREAD_WA_SIZE);
static THD_FUNCTION(RxThread, arg) {
  chRegSetThreadName("MavRx");
  msg_t c = Q_TIMEOUT;
  mavChannel *channel = static_cast<mavChannel *>(arg);

  while (!chThdShouldTerminateX()) {
    c = channel->get(MS2ST(20));
    if (c != Q_TIMEOUT){
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &rx_msg, &rx_status)) {
        MavPostman::spam_list.dispatch(rx_msg);
      }
    }
  }

  chThdExit(MSG_OK);
}

/**
 *
 */
static THD_WORKING_AREA(TxThreadWA, WORKER_TX_THREAD_WA_SIZE);
static THD_FUNCTION(TxThread, arg) {
  chRegSetThreadName("MavTx");
  mavChannel *channel = static_cast<mavChannel *>(arg);
  mavMail *mail;
  size_t len;

  while (!chThdShouldTerminateX()) {
    if (MSG_OK == txmb.fetch(&mail, MS2ST(100))) {
      if (0 != mavlink_encode(mail->msgid, mail->compid, &tx_msg,  mail->mavmsg)) {
        len = mavlink_msg_to_send_buffer(sendbuf, &tx_msg);
        channel->write(sendbuf, len);
      }
      mail->release();
    }
  }

  chThdExit(MSG_OK);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
MavPostman::MavPostman(void){
  memset(&rx_msg, 0, sizeof(rx_msg));
  memset(&rx_status, 0, sizeof(rx_status));
  memset(&tx_msg, 0, sizeof(tx_msg));
}

/**
 *
 */
void MavPostman::start(mavChannel *chan){

  this->channel = chan;

  rxworker = chThdCreateStatic(RxThreadWA, sizeof(RxThreadWA),
      MAVPOSTMANPRIO, RxThread, channel);
  osalDbgAssert(nullptr != rxworker, "Can not allocate memory");

  txworker = chThdCreateStatic(TxThreadWA, sizeof(TxThreadWA),
      MAVPOSTMANPRIO, TxThread, channel);
  osalDbgAssert(nullptr != txworker, "Can not allocate memory");

  ready = true;
}

/**
 *
 */
void MavPostman::stop(void){

  if (false == ready)
    return;
  else {
    ready = false;
    chThdTerminate(rxworker);
    chThdTerminate(txworker);
    chThdWait(rxworker);
    chThdWait(txworker);
    rxworker = nullptr;
    txworker = nullptr;
    channel = nullptr;
  }
}

/**
 *
 */
msg_t MavPostman::post(mavMail &mail) {
  msg_t ret = MSG_RESET;

  if (ready == true)
    ret = txmb.post(&mail, TIME_IMMEDIATE);

  return ret;
}

/**
 *
 */
msg_t MavPostman::postAhead(mavMail &mail) {
  msg_t ret = MSG_RESET;

  if (ready == true)
    ret = txmb.postAhead(&mail, TIME_IMMEDIATE);

  return ret;
}

/**
 *
 */
void MavPostman::free(mavlink_message_t *msg) {
  spam_list.free(msg);
}

/**
 *
 */
void MavPostman::subscribe(uint8_t msg_id, SubscribeLink *sl){
  spam_list.subscribe(msg_id, sl);
}

/**
 *
 */
void MavPostman::unsubscribe(uint8_t msg_id, SubscribeLink *sl){
  spam_list.unsubscribe(msg_id, sl);
}
