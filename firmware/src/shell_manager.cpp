#include "main.h"

#include "global_flags.h"
#include "cli.hpp"
#include "param_registry.hpp"
#include "usbcfg.hpp"

#include "shell_manager.hpp"
#include "usb_debouncer.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define DEBOUNCE_TIMEOUT     MS2ST(100)

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
static UsbDebouncer debouncer;

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
static void shell_reschedule(bool plugstate){

  if (true == plugstate){
    /* Activates the USB driver and then the USB bus pull-up on D+. */
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
    SpawnShellThreads(&SDU1);
  }
  else{
    KillShellThreads();
    usbDisconnectBus(serusbcfg.usbp);
    usbStop(serusbcfg.usbp);
  }
}

/**
 * Track changes of sh_overxbee flag and fork appropriate threads
 */
static WORKING_AREA(ShellManagerWA, 128);
static msg_t ShellManagerThread(void *arg){
  (void)arg;
  chRegSetThreadName("ShellManager");
  bool plugstate = false;
  bool plugstate_prev = false;

  while (!chThdShouldTerminate()) {
    chThdSleep(DEBOUNCE_TIMEOUT);
    plugstate = debouncer.update();

    if (plugstate != plugstate_prev){
      shell_reschedule(plugstate);
      plugstate_prev = plugstate;
    }
  }

  /* cleaning */
  chThdExit(0);
  return 0;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
ShellManager::ShellManager(void){
  this->state = SHELL_MANAGER_UNINIT;
  this->worker = NULL;
}

/**
 *
 */
void ShellManager::start(void){

  /* Initializes a serial-over-USB CDC driver. */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /* start shell keeper thread */
  this->worker = chThdCreateStatic(ShellManagerWA, sizeof(ShellManagerWA),
                                   NORMALPRIO, ShellManagerThread, NULL);

  chDbgCheck(NULL != this->worker, "Can not allocate memory");
}

/**
 *
 */
void ShellManager::stop(void){
  this->state = SHELL_MANAGER_STOP;
  chThdTerminate(this->worker);
  chThdWait(this->worker);
  sduStop(&SDU1);
}

