#include <stdio.h>
#include <string.h>

#include "main.h"
#include "global_flags.h"
#include "message.hpp"
#include "exti_local.hpp"
#include "pads.h"
#include "imcuc_channel.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
//115200, 230400, 460800, 921600, 1843200
#define BAUDRATE_IMCUC  2000000

/* time to sleep in every 'write' call */
#define FLOOD_PROTECT_PAUSE     MS2ST(1)

/* convenience wrappers */
#define acquire()   {imcucsem.wait();}
#define release()   {imcucsem.signal();}
#define releaseI()  {imcucsem.signalI();}

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavChannelImcuc imcuc_channel;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */
static void txend1(UARTDriver *uartp);
static void txend2(UARTDriver *uartp);
static void rxerr(UARTDriver *uartp, uartflags_t e);
static void rxchar(UARTDriver *uartp, uint16_t c);
static void rxend(UARTDriver *uartp);

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
/*
 * UART driver configuration structure.
 */
static const UARTConfig imcuc_uart_cfg = {
  txend1,
  txend2,
  rxend,
  rxchar,
  rxerr,
  BAUDRATE_IMCUC,
  0,
  0,
  0
};

/*
 * implements mutual access to transmission channel
 */
static chibios_rt::BinarySemaphore imcucsem(false);

mavChannelImcuc imcuc_channel(&UART_IMCUC, &imcuc_uart_cfg);

static systime_t prev_txend2 = 0;

static size_t missed_interrupt = 0;
static size_t overrun_error = 0;
static size_t other_error = 0;
static size_t rxchar_error = 0;
static size_t rxend_error = 0;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */
/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
  (void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
  (void)uartp;

  chSysLockFromIsr();
  if(UART_TX_ACTIVE == uartp->txstate){
    uartStopSendI(uartp);
    missed_interrupt++;
  }
  prev_txend2 = chTimeNow();
  releaseI();           /* release transmit buffer */
  imcuc_set_pad();      /* signalizes receiving end point */
  chSysUnlockFromIsr();
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
  (void)uartp;

  if (UART_OVERRUN_ERROR == e)
    overrun_error++;
  else
    other_error++;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
  (void)uartp;
  (void)c;
  rxchar_error++;
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
  (void)uartp;

  chSysLockFromIsr();
  imcuc_channel.ImcucIsr();
  chSysUnlockFromIsr();
  rxend_error++;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
mavChannelImcuc::mavChannelImcuc(UARTDriver *uartp, const UARTConfig *uart_cfg):
mavChannelUart(uartp, uart_cfg)
{
  rxbytes = 0;
  txbytes = 0;
}

/**
 *
 */
void mavChannelImcuc::start(void){
  chDbgCheck(GlobalFlags.exti_ready == 1, "exti not ready");
  imcuc_set_pad();
  uartStart(uartp, uart_cfg);
  uartStartReceive(uartp, MAVCHANNEL_BUF_SIZE, multi_rxbuf.current());
  Exti.Imcuc(true);
  GlobalFlags.imcuc_ready = 1;
}

/**
 *
 */
void mavChannelImcuc::stop(void){
  GlobalFlags.imcuc_ready = 0;
  Exti.Imcuc(false);
  imcuc_clear_pad();
  uartStop(uartp);
}

/**
 *
 */
void mavChannelImcuc::write(const uint8_t *buf, uint16_t len){
  (void)buf;
  (void)len;

  txbytes += len;
  acquire();

  /* flooding prevention of receiving part */
  if ((chTimeNow() - prev_txend2) < FLOOD_PROTECT_PAUSE){
    chThdSleep(FLOOD_PROTECT_PAUSE);
  }

  imcuc_clear_pad(); // this pad will be set back in tx end callback
  //halPolledDelay(168 * 10);
  uartStartSend(uartp, len, buf);
}

/**
 * Designed to be called from ISR.
 */
void mavChannelImcuc::ImcucIsr(void){
  rxbytes += uartStopReceiveI(uartp);

  /* switch to next buffer only if there is some space in message box */
  if (0 != rxmb.getFreeCountI()){
    rxmb.postI((msg_t)multi_rxbuf.current());
    uartStartReceiveI(uartp, MAVCHANNEL_BUF_SIZE, multi_rxbuf.next());
  }
  else{
    uartStartReceiveI(uartp, MAVCHANNEL_BUF_SIZE, multi_rxbuf.current());
  }
}
