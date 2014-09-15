#include "main.h"
#include "mavworker.hpp"
#include "mavpostman.hpp"
#include "multi_buffer.hpp"

#include "this_comp_id.h"
#include "encode_table.h"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define THIS_SYS_ID     20

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

MavWorker mav_worker;

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
static mavlink_status_t status;
static chibios_rt::Mailbox<mavMail*, 12> txmb;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 *
 */
static THD_WORKING_AREA(RxThreadWA, WORKER_RX_THREAD_WA_SIZE);
static THD_FUNCTION(RxThread, arg) {
  chRegSetThreadName("MavRx");
  msg_t c = Q_TIMEOUT;
  mavChannel *channel = static_cast<mavChannel *>(arg);
  mavlink_message_t mavlink_message_struct;

  while (!chThdShouldTerminateX()){
    c = channel->get(MS2ST(50));
    if (c != Q_TIMEOUT){
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &mavlink_message_struct, &status)) {
        mav_postman.dispatch(mavlink_message_struct);
      }
    }
  }

  chThdExit(MSG_OK);
  return MSG_OK;
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
  mavlink_message_t mavlink_message_struct;
  uint8_t sendbuf[MAVLINK_SENDBUF_SIZE];

  while (!chThdShouldTerminateX()){
    if (MSG_OK == txmb.fetch(&mail, MS2ST(20))){
      if (0 != mavlink_encode_table[mail->msgid](THIS_SYS_ID, mail->compid, &mavlink_message_struct, mail->mavmsg)){
        len = mavlink_msg_to_send_buffer(sendbuf, &mavlink_message_struct);
      }
      (void)channel;
      (void)len;
      //channel->write(sendbuf, len);
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
MavWorker::MavWorker(void){
  return;
}

/**
 *
 */
void MavWorker::start(mavChannel *chan){

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
void MavWorker::stop(void){

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
void MavWorker::subscribe(uint8_t msg_id, SubscribeLink *sl){
  osalDbgCheck(ready == true);
  mav_postman.add_link(msg_id, sl);
}

/**
 *
 */
void MavWorker::unsubscribe(uint8_t msg_id, SubscribeLink *sl){
  osalDbgCheck(ready == true);
  mav_postman.del_link(msg_id, sl);
}

/**
 *
 */
msg_t MavWorker::post(mavMail &mail){
  msg_t ret = MSG_RESET;

  if (ready == true){
    ret = txmb.post(&mail, TIME_IMMEDIATE);
  }

  return ret;
}




