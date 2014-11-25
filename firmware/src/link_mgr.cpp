#include "main.h"

#include "param_registry.hpp"
#include "usbcfg.h"
#include "usb_debouncer.hpp"
#include "link_mgr.hpp"

#include "shell.hpp"
#include "mav_channel_serial.hpp"
#include "mav_channel_usbserial.hpp"
#include "mav_postman.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define BAUDRATE_XBEE           115200
#define DEBOUNCE_TIMEOUT        MS2ST(100)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static SerialConfig xbee_ser_cfg = {
    BAUDRATE_XBEE,
    0,
    0,
    0//USART_CR3_CTSE | USART_CR3_RTSE
};

static const uint32_t *sh_overxbee;

static UsbDebouncer debouncer;

static mavChannelSerial channel_serial(&XBEESD, &xbee_ser_cfg);
static mavChannelUsbSerial channel_usb_serial(&SDU1, &serusbcfg);

static Shell shell;

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
/**
 *
 */
static void boot_strap(bool plug_prev, uint32_t sh_prev) {
  if (0 == sh_prev){
    mav_postman.start(&channel_serial);
    if (true == plug_prev){
      sduStart(&SDU1, &serusbcfg);
      usbStart(serusbcfg.usbp, &usbcfg);
      usb_lld_connect_bus_workaround();
      usbConnectBus(serusbcfg.usbp);
      osalThreadSleepMilliseconds(500);
      shell.start((SerialDriver *)&SDU1);
    }
  }
  else{
    sdStart(&XBEESD, &xbee_ser_cfg);
    shell.start(&XBEESD);
    if (true == plug_prev){
      usbStart(serusbcfg.usbp, &usbcfg);
      usb_lld_connect_bus_workaround();
      usbConnectBus(serusbcfg.usbp);
      osalThreadSleepMilliseconds(500);
      mav_postman.start(&channel_usb_serial);
    }
  }
}

/**
 * Track changes of sh_overxbee flag and fork appropriate threads
 */
static THD_WORKING_AREA(LinkMgrThreadWA, 128);
static THD_FUNCTION(LinkMgrThread, arg) {
  (void)arg;
  chRegSetThreadName("LinkMgr");

  uint32_t sh_prev; // cached previous value
  uint32_t tmp_sh;
  bool plug_prev;
  bool tmp_plug;

  plug_prev = debouncer.update();
  /* Activates the USB driver and then the USB bus pull-up on D+.
     Note, a delay is inserted in order to not have to disconnect the cable
     after a reset. */
  usb_lld_disconnect_bus_workaround();
  usbDisconnectBus(serusbcfg.usbp);
  osalThreadSleepMilliseconds(1000);

  plug_prev = debouncer.update();
  sh_prev = *sh_overxbee;
  boot_strap(plug_prev, sh_prev);

  /* now track changes of flag and fork appropriate threads */
  while (!chThdShouldTerminateX()) {
    tmp_plug = debouncer.update();
    tmp_sh = *sh_overxbee;

    if ((tmp_plug != plug_prev) || (tmp_sh != sh_prev)) {
      plug_prev = tmp_plug;
      sh_prev = tmp_sh;

      mav_postman.stop();
      shell.stop();
      usb_lld_disconnect_bus_workaround();
      usbDisconnectBus(serusbcfg.usbp);
      osalThreadSleep(DEBOUNCE_TIMEOUT);
      usbStop(serusbcfg.usbp);
      osalThreadSleep(DEBOUNCE_TIMEOUT);
      sduStop(&SDU1);
      osalThreadSleep(DEBOUNCE_TIMEOUT);
      sdStop(&XBEESD);

      boot_strap(plug_prev, sh_prev);
    }
    osalThreadSleep(DEBOUNCE_TIMEOUT);
  }

  chThdExit(MSG_OK);
  return MSG_OK;
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
                                    LINKPRIO, LinkMgrThread, NULL);
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
