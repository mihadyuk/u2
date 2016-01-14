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
#if XBEE_USE_CTS_RTS
    USART_CR3_CTSE | USART_CR3_RTSE
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
  /* Activates the USB driver and then the USB bus pull-up on D+.
     Note, a delay is inserted in order to not have to disconnect the cable
     after a reset. */
  usb_lld_disconnect_bus_workaround();
  usbDisconnectBus(serusbcfg.usbp);
  osalThreadSleepMilliseconds(1000);

  sh_now = sh_prev = *sh_overxbee;
  plug_now = debouncer.update();
  plug_prev = !plug_now; /* provocate state updating */
  sdStart(&XBEESD, &xbee_ser_cfg);
  /* usb will be started once and forever because of some strange bugs in
   * stopping sequence */
  usbStart(serusbcfg.usbp, &usbcfg);

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
          sduStart(&SDU1, &serusbcfg);
          usb_lld_connect_bus_workaround();
          usbConnectBus(serusbcfg.usbp);
          osalThreadSleepMilliseconds(500);
        }
        else {
          usbDisconnectBus(serusbcfg.usbp);
          usb_lld_disconnect_bus_workaround();
          sduStop(&SDU1);
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
  usbDisconnectBus(serusbcfg.usbp);
  usb_lld_disconnect_bus_workaround();
  usbStop(serusbcfg.usbp);
  sduStop(&SDU1);
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
  sduStart(&SDU1, &serusbcfg); // workaround against stopping of non started driver
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
  sduStop(&SDU1);
  sdStop(&XBEESD);
}
