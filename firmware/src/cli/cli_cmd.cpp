#include <string.h>
#include <stdio.h>

#include "main.h"
#include "chprintf.h"

#include "cli.hpp"
#include "cli_cmd.hpp"

#include "firmware_version.h"

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
extern memory_heap_t ThdHeap;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
static thread_t *loop_clicmd_tp;

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
thread_t* clear_clicmd(int argc, const char * const * argv, BaseChannel *bchnp){
  (void)bchnp;
  (void)argc;
  (void)argv;
  cli_print("\033[2J");    // ESC seq for clear entire screen
  cli_print("\033[H");     // ESC seq for move cursor at left-top corner
  return NULL;
}

/**
 *
 */
thread_t* echo_clicmd(int argc, const char * const * argv, BaseChannel *bchnp){
  (void)bchnp;

  int i = 0;
  while (i < argc)
    cli_print(argv[i++]);

  cli_print(ENDL);
  return NULL;
}

/**
 *
 */
static THD_WORKING_AREA(LoopCmdThreadWA, 1024);
static THD_FUNCTION(LoopCmdThread, arg) {
  chRegSetThreadName("LoopCmd");
  (void)arg;

  cli_print("This is loop function test. Press ^C to stop it.\n\r");
  while (!chThdShouldTerminateX()){
    int n = 16;
    char str[n];
    snprintf(str, n, "%i\r\n", -666);
    cli_print(str);
    chThdSleepMilliseconds(25);
  }

  chThdExit(MSG_OK);
}

/**
 *
 */
thread_t* loop_clicmd(int argc, const char * const * argv, BaseChannel *bchnp){
  (void)bchnp;
  (void)argc;
  (void)argv;

  loop_clicmd_tp = chThdCreateFromHeap(&ThdHeap,
                                  sizeof(LoopCmdThreadWA),
                                  NORMALPRIO + 5,
                                  LoopCmdThread,
                                  NULL);

  if (loop_clicmd_tp == NULL)
    osalSysHalt("can not allocate memory");

  return loop_clicmd_tp;
}

/**
 *
 */
thread_t* reboot_clicmd(int argc, const char * const * argv, BaseChannel *bchnp){
  (void)bchnp;
  (void)argv;
  (void)argc;
  cli_print("System going to reboot now...\r\n");
  chThdSleepMilliseconds(100);
  NVIC_SystemReset();
  return NULL;
}

/**
 *
 */
thread_t* sleep_clicmd(int argc, const char * const * argv, BaseChannel *bchnp){
  (void)bchnp;
  (void)argv;
  (void)argc;

  cli_print("System sleeping.\r\n");
  cli_print("Press any key to wake it up.\r\n");
  chThdSleepMilliseconds(100);

  chSysLock();
  PWR->CR |= (PWR_CR_PDDS | PWR_CR_LPDS | PWR_CR_CSBF | PWR_CR_CWUF);
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __WFI();
  return NULL;
}

/**
 *
 */
thread_t* selftest_clicmd(int argc, const char * const * argv, BaseChannel *bchnp){
  (void)bchnp;
  (void)argv;
  (void)argc;

  cli_print("GPS - OK\r\nModem - OK\r\nEEPROM - OK\r\nStorage - OK\r\nServos - OK\r\n");
  return NULL;
}

/**
 *
 */
thread_t* uname_clicmd(int argc, const char * const * argv, BaseChannel *bchnp){
  (void)argc;
  (void)argv;

  BaseSequentialStream* chp = (BaseSequentialStream*)bchnp;

  chprintf(chp, "Kernel:       %s\r\n", CH_KERNEL_VERSION);
#ifdef PORT_COMPILER_NAME
  chprintf(chp, "Compiler:     %s\r\n", PORT_COMPILER_NAME);
#endif
  chprintf(chp, "Architecture: %s\r\n", PORT_ARCHITECTURE_NAME);
#ifdef PORT_CORE_VARIANT_NAME
  chprintf(chp, "Core Variant: %s\r\n", PORT_CORE_VARIANT_NAME);
#endif
#ifdef PORT_INFO
  chprintf(chp, "Port Info:    %s\r\n", PORT_INFO);
#endif
#ifdef PLATFORM_NAME
  chprintf(chp, "Platform:     %s\r\n", PLATFORM_NAME);
#endif
#ifdef BOARD_NAME
  chprintf(chp, "Board:        %s\r\n", BOARD_NAME);
#endif
#ifdef __DATE__
#ifdef __TIME__
  chprintf(chp, "Build time:   %s%s%s\r\n", __DATE__, " - ", __TIME__);
#endif
#endif
  cli_println("GIT info:     ");
  cli_println(commit_hash);
  cli_println(commit_date);
  return NULL;
}

/**
 *
 */
thread_t* ps_clicmd(int argc, const char * const * argv, BaseChannel *bchnp){
  (void)bchnp;
  (void)argc;
  (void)argv;
  thread_t *curr = NULL;

#if !CH_CFG_USE_REGISTRY
  (void)curr;
  cli_println("In order to use this function you must set CH_USE_REGISTRY -> TRUE");
  return NULL;
#else
  curr = chRegFirstThread();

  cli_print("name\t\tstate\tprio\ttime\n\r");
  cli_print("------------------------------------------\n\r");
  while (curr->p_refs > 0){
    cli_print(curr->p_name);
    cli_print("\t");
    //cli_print(curr->p_state);
    //cli_print(curr->p_prio);
    //cli_print(curr->p_time);
    cli_print("\n\r");
    curr = chRegNextThread(curr);
  }
  return NULL;
#endif
}





