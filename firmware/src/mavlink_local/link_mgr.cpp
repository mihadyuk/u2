#include "main.h"

#include "param_registry.hpp"
#include "usbcfg.h"
#include "debouncer.hpp"
#include "link_mgr.hpp"

#include "cli.hpp"
#include "mav_channel_serial.hpp"
#include "mav_channel_usbserial.hpp"
#include "mav_postman.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define DEBOUNCE_PERIOD         MS2ST(50)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static const SerialConfig xbee_ser_cfg = {
    XBEE_BAUDRATE,
    0,
    0,
#if XBEE_USE_HW_FLOW
    USART_CR3_RTSE
#else
    0
#endif
};

static const uint32_t *sh_overxbee;

static Debouncer debouncer(2, 0, usb_lld_plug_state);

static mavChannelSerial channel_serial;
static mavChannelUsbSerial channel_usb_serial;

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

static void usb_serial_up(void) {
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usb_lld_connect_bus_workaround();
  chThdSleepMilliseconds(1);
  usbConnectBus(serusbcfg.usbp);
}

static void usb_serial_down(void) {
  usb_lld_disconnect_bus_workaround();
  chThdSleepMilliseconds(1);
  usbDisconnectBus(serusbcfg.usbp);
  usbStop(serusbcfg.usbp);
  sduStop(&SDU1);
}

/**
 * Track changes of sh_overxbee flag and fork appropriate threads
 */
static THD_WORKING_AREA(LinkMgrThreadWA, 128);
static THD_FUNCTION(LinkMgrThread, arg) {
  (void)arg;
  chRegSetThreadName("LinkMgr");

  uint32_t sh_prev; // cached previous value
  uint32_t sh_now;
  int plug_prev;
  int plug_now;

  plug_prev = debouncer.update();

  sh_now = sh_prev = *sh_overxbee;
  plug_now = debouncer.update();
  plug_prev = !plug_now; /* provocate state updating */
  sdStart(&XBEESD, &xbee_ser_cfg);

  /* now track changes of flag and fork appropriate threads */
  while (!chThdShouldTerminateX()) {

    /* first process plug state changes */
    if ((plug_now != plug_prev) || (sh_now != sh_prev)) {

      /* start from clean state */
      KillShellThreads();
      mav_postman.stop();
      channel_serial.stop();
      channel_usb_serial.stop();

      /* start or stop USB driver */
      if (plug_now != plug_prev) {
        if (1 == plug_now) {
          usb_serial_up();
        }
        else {
          usb_serial_down();
        }
      }

      /* now process shell flag */
      if (1 == sh_now) {
        if (1 == plug_now) {
          channel_usb_serial.start(&SDU1);
          mav_postman.start(&channel_usb_serial);
        }
        SpawnShellThreads(&XBEESD);
      }
      else {
        if (1 == plug_now) {
          SpawnShellThreads(&SDU1);
        }
        channel_serial.start(&XBEESD);
        mav_postman.start(&channel_serial);
      }
    }

    /* process flags for next iteration */
    plug_prev = plug_now;
    sh_prev = sh_now;
    osalThreadSleep(DEBOUNCE_PERIOD);
    plug_now = debouncer.update();
    sh_now = *sh_overxbee;
  }

  KillShellThreads();
  mav_postman.stop();
  channel_serial.stop();
  channel_usb_serial.stop();
  usb_serial_down();
  sdStop(&XBEESD);

  chThdExit(MSG_OK);
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
LinkMgr::LinkMgr(void){
  return;
}

/**
 *
 */
void LinkMgr::start(void){
  param_registry.valueSearch("SH_over_radio", &sh_overxbee);
  sduObjectInit(&SDU1);

  this->worker = chThdCreateStatic(LinkMgrThreadWA, sizeof(LinkMgrThreadWA),
      SHELLPRIO, LinkMgrThread, NULL);
  osalDbgAssert(NULL != this->worker, "can not allocate memory");
}

/**
 *
 */
void LinkMgr::stop(void){
  chThdTerminate(this->worker);
  chThdWait(this->worker);
}
