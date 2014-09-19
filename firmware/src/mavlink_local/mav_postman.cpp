#include "main.h"
#include "mav_postman.hpp"
#include "mav_spam_list.hpp"
#include "multi_buffer.hpp"
#include "mav_encode.h"

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
//static mavlink_status_t status;
static chibios_rt::Mailbox<mavMail*, 12> txmb;
//MavSpamList MavPostman::spam_list;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

void mavPostmanRxLoop(uint8_t c);

/**
 *
 */
static THD_WORKING_AREA(RxThreadWA, WORKER_RX_THREAD_WA_SIZE);
static THD_FUNCTION(RxThread, arg) {
  chRegSetThreadName("MavRx");
  msg_t c = Q_TIMEOUT;
  mavChannel *channel = static_cast<mavChannel *>(arg);
//  mavlink_message_t rx_msg;

  while (!chThdShouldTerminateX()){
    c = channel->get(MS2ST(20));
    if (c != Q_TIMEOUT){
      mavPostmanRxLoop(c);
//      if (mavlink_parse_char(MAVLINK_COMM_0, c, &rx_msg, &status)) {
//        MavPostman::spam_list.dispatch(rx_msg);
//      }
    }
  }

  chThdExit(MSG_OK);
  return MSG_OK;
}

/**
 *
 */
static mavlink_message_t selftest_msg;
static mavlink_status_t selftest_status;

static THD_WORKING_AREA(TxThreadWA, WORKER_TX_THREAD_WA_SIZE);
static THD_FUNCTION(TxThread, arg) {
  chRegSetThreadName("MavTx");
  mavChannel *channel = static_cast<mavChannel *>(arg);
  mavMail *mail;
  size_t len;
  mavlink_message_t tx_msg;
  uint8_t sendbuf[MAVLINK_SENDBUF_SIZE];

  while (!chThdShouldTerminateX()){
    if (MSG_OK == txmb.fetch(&mail, MS2ST(100))){
      if (0 != mavlink_encode(mail->msgid, &tx_msg,  mail->mavmsg)){
        len = mavlink_msg_to_send_buffer(sendbuf, &tx_msg);

        uint8_t ret2 = 0;
        for (size_t i=0; i<len; i++){
          ret2 = mavlink_parse_char(MAVLINK_COMM_0, sendbuf[i], &selftest_msg, &selftest_status);
        }
        osalDbgCheck(1 == ret2);

        channel->write(sendbuf, len);

        uint8_t ret = 0;
        for (size_t i=0; i<len; i++){
          ret = mavlink_parse_char(MAVLINK_COMM_0, sendbuf[i], &selftest_msg, &selftest_status);
        }
        osalDbgCheck(1 == ret);


      }
      mail->release();
    }
  }

  chThdExit(MSG_OK);
  return MSG_OK;
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
  return;
}

/**
 *
 */
void MavPostman::start(mavChannel *chan){

  this->channel = chan;
  this->channel->start();

  rxworker = chThdCreateStatic(RxThreadWA, sizeof(RxThreadWA),
      LINKPRIO, RxThread, channel);
  osalDbgAssert(NULL != rxworker, "Can not allocate memory");
  txworker = chThdCreateStatic(TxThreadWA, sizeof(TxThreadWA),
      LINKPRIO, TxThread, channel);
  osalDbgAssert(NULL != txworker, "Can not allocate memory");

  ready = true;
}

/**
 *
 */
void MavPostman::stop(void){

  ready = false;

  chThdTerminate(rxworker);
  chThdTerminate(txworker);

  chThdWait(rxworker);
  chThdWait(txworker);

  rxworker = NULL;
  txworker = NULL;

  channel->stop();
  channel = NULL;
}

/**
 *
 */
msg_t MavPostman::post(mavMail &mail){
  msg_t ret = MSG_RESET;

  if (ready == true)
    ret = txmb.post(&mail, TIME_IMMEDIATE);

  return ret;
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
