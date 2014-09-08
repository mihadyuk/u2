#include "main.h"
#include "mavworker.hpp"

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

  while (!chThdShouldTerminateX()){
    c = channel->getTimeout(MS2ST(50));
    if (c != Q_TIMEOUT){
      //link_cbc_parse((uint8_t)c);
      osalThreadSleepMilliseconds(10);
    }
  }

  chThdExit(0);
  return 0;
}

/**
 *
 */
static THD_WORKING_AREA(TxThreadWA, WORKER_TX_THREAD_WA_SIZE);
static THD_FUNCTION(TxThread, arg) {
  chRegSetThreadName("MavTx");
  mavChannel *channel = static_cast<mavChannel *>(arg);

  //link_cbc_pack_cycle(channel);

  while (!chThdShouldTerminateX()){
    (void)channel;
    osalThreadSleepMilliseconds(10);
  }

  chThdExit(0);
  return 0;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
MavWorker::MavWorker(void){
  this->rxworker = NULL;
  this->txworker = NULL;
}

/**
 *
 */
void MavWorker::start(mavChannel *channel){
  channel->start();

  rxworker = chThdCreateStatic(RxThreadWA, sizeof(RxThreadWA),
      LINKPRIO, RxThread, channel);
  osalDbgAssert(NULL != rxworker, "Can not allocate memory");
  txworker = chThdCreateStatic(TxThreadWA, sizeof(TxThreadWA),
      LINKPRIO, TxThread, channel);
  osalDbgAssert(NULL != txworker, "Can not allocate memory");
}

/**
 *
 */
void MavWorker::stop(void){

  chThdTerminate(rxworker);
  chThdTerminate(txworker);

  chThdWait(rxworker);
  chThdWait(txworker);

  channel->stop();
  channel = NULL;
}

void MavWorker::subscribe(uint8_t msg_id, SubscribeLink *sl){
  mav_spammer.add_link(msg_id, sl);
}

void MavWorker::unsubscribe(uint8_t msg_id, SubscribeLink *sl){
  mav_spammer.del_link(msg_id, sl);
}
