#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "microrl.h"

#include "main.h"
#include "global_flags.h"
#include "cli.hpp"
#include "cli_cmd.hpp"
#include "param_registry.hpp"
#include "param_cli.hpp"
#include "time_keeper.hpp"
#include "hil_cli.hpp"
#include "gnss_cli.hpp"
//#include "irq_storm.hpp"
//#include "cli_cal.hpp"
//#include "servo_cli.hpp"
//#include "sensors_cli.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define _NUM_OF_CMD (sizeof(cliutils)/sizeof(ShellCmd_t))

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern GlobalFlags_t GlobalFlags;
extern ParamRegistry param_registry;

/*
 *******************************************************************************
 * PROTOTYPES
 *******************************************************************************
 */
static thread_t* shellswap_clicmd(int argc, const char * const * argv, BaseChannel *bchnp);
static thread_t* help_clicmd(int argc, const char * const * argv, BaseChannel *bchnp);

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static const ShellCmd_t cliutils[] = {
    {"clear",     &clear_clicmd,      "clear screen"},
//    {"cal",       &cal_clicmd,        "start calibration of onboard sensors"},
    {"date",      &date_clicmd,       "print and set current date"},
//    {"dcm",       &dcm_clicmd,        "print DCM in realtime until ^C pressed"},
    {"gnss",      &gnss_clicmd,       "debug interface for GPS receiver"},
    {"echo",      &echo_clicmd,       "echo it's input to terminal"},
    {"help",      &help_clicmd,       "this message"},
    {"hil",       &hil_clicmd,        "HIL applet for manual checks"},
    {"info",      &uname_clicmd,      "system information"},
//    {"irqstorm",  &irqstorm_clicmd,   "run longterm stability load test"},
    {"loop",      &loop_clicmd,       "command to test ^C fucntionallity"},
    {"param",     &param_clicmd,      "manage onboard system paramters"},
    {"ps",        &ps_clicmd,         "info about running threads"},
    {"reboot",    &reboot_clicmd,     "reboot system. Use with caution!"},
    {"selftest",  &selftest_clicmd,   "exectute selftests (stub)"},
//    {"sensors",   &sensors_clicmd,    "get human readable data from onboard sensors"},
//    {"servo",     &servo_clicmd,      "change actuators' state during servo limits tuning"},
    {"sleep",     &sleep_clicmd,      "put autopilot board in sleep state (do not use it)"},
    {"shellswap", &shellswap_clicmd,  "swap telemetry and shell channels"},
    {"uname",     &uname_clicmd,      "'info' alias"},
//    {"wps",       &wps_clicmd,        "simple waypoint interface"},
    {NULL,        NULL,               NULL}/* end marker */
};

// array for comletion
static char *compl_world[_NUM_OF_CMD + 1];

/* thread pointer to currently executing command */
static thread_t *current_cmd_tp = NULL;

/* pointer to shell thread */
static thread_t *shell_tp = NULL;

/* serial interface for shell */
static BaseChannel *ShellChnp;

/*
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 */
/**
 * Print routine for microrl.
 */
static void microrl_print(void* user_halde, const char *str){
  (void)user_halde;
  cli_print(str);
}

/**
 * Search value (pointer to function) by key (name string)
 */
static int32_t cmd_search(const char* key, const ShellCmd_t *cmdarray){
  uint32_t i = 0;

  while (cmdarray[i].name != NULL){
    if (strcmp(key, cmdarray[i].name) == 0)
      return i;
    i++;
  }
  return -1;
}

//*****************************************************************************
// execute callback for microrl library
// do what you want here, but don't write to argv!!! read only!!
static unsigned int execute (void* user_handle, int argc, const char * const * argv){
  (void)user_handle;

  int i = 0;

  /* search first token */
  i = cmd_search(argv[0], cliutils);
  if (i == -1){
    cli_print ("command: '");
    cli_print ((char*)argv[0]);
    cli_print ("' Not found.\n\r");
  }
  else{
    if (argc > 1)
      current_cmd_tp = cliutils[i].func(argc - 1, &argv[1], ShellChnp);
    else
      current_cmd_tp = cliutils[i].func(0, NULL, ShellChnp);
  }
  return 0;
}

//*****************************************************************************
// completion callback for microrl library
#ifdef _USE_COMPLETE
static char ** complete(void* user_handle, int argc, const char * const * argv) {
  (void)user_handle;

  int j = 0;
  int i = 0;

  compl_world[0] = NULL;

  // if there is token in cmdline
  if (argc == 1) {
    // get last entered token
    char * bit = (char*)argv [argc-1];
    // iterate through our available token and match it
    while (cliutils[i].name != NULL){
      if (strstr(cliutils[i].name, bit) == cliutils[i].name)
        compl_world[j++] = (char *)cliutils[i].name;
      i++;
    }
  }
  else { // if there is no token in cmdline, just print all available token
    while (cliutils[j].name != NULL){
      compl_world[j] = (char *)cliutils[j].name;
      j++;
    }
  }

  // note! last ptr in array always must be NULL!!!
  compl_world[j] = NULL;
  // return set of variants
  return compl_world;
}
#endif

/**
 *
 */
static void sigint (void* user_handle){
  (void)user_handle;

  if (current_cmd_tp != NULL){
    cli_print("^C pressed. Exiting...");
    chThdTerminate(current_cmd_tp);
    chThdWait(current_cmd_tp);
    current_cmd_tp = NULL;
    cli_print("--> Done. Press 'Enter' to return to shell");
  }
}

/**
 * Thread function
 */
static THD_WORKING_AREA(ShellThreadWA, 2048);
static THD_FUNCTION(ShellThread, arg) {

  chRegSetThreadName("Shell");

  /* init static pointer for serial driver with received pointer */
  ShellChnp = (BaseChannel *)arg;

  // create and init microrl object
  microrl_t microrl_shell;
  chThdSleepMilliseconds(10);
  cli_print("Mobile Operational System Kamize (MOSK) welcomes you.");
  cli_print(ENDL);
  chThdSleepMilliseconds(10);
  cli_print("Press enter to get command prompt.");
  microrl_init(&microrl_shell, NULL, microrl_print);

  // set callback for execute
  microrl_set_execute_callback(&microrl_shell, execute);

  // set callback for completion (optionally)
  microrl_set_complete_callback(&microrl_shell, complete);

  // set callback for ctrl+c handling (optionally)
  microrl_set_sigint_callback(&microrl_shell, sigint);

  setGlobalFlag(GlobalFlags.shell_ready);

  while (!chThdShouldTerminateX()){
    // put received char from stdin to microrl lib
    msg_t c = chnGetTimeout(ShellChnp, MS2ST(50));
    if (c != Q_TIMEOUT)
      microrl_insert_char(&microrl_shell, (char)c);

    /* if fork finished than collect allocated for it memory */
    if ((current_cmd_tp != NULL) && (current_cmd_tp->p_state == CH_STATE_FINAL)){
      chThdWait(current_cmd_tp);
      current_cmd_tp = NULL;
    }
  }

  /* умираем по всем правилам, не забываем убить потомков */
  if (current_cmd_tp != NULL){
    if (current_cmd_tp->p_state != CH_STATE_FINAL)
      chThdTerminate(current_cmd_tp);
    chThdWait(current_cmd_tp);
  }

  clearGlobalFlag(GlobalFlags.shell_ready);
  chThdExit(MSG_OK);
}

/**
 *
 */
static thread_t* shellswap_clicmd(int argc, const char * const * argv, BaseChannel *bchnp){
  (void)bchnp;
  (void)argc;
  (void)argv;

  uint32_t *sh_flag;
  param_registry.valueSearch("SH_over_radio", &sh_flag);

  if (0 == *sh_flag)
    *sh_flag = 1;
  else
    *sh_flag = 0;

  param_registry.syncParam("SH_over_radio");
  return NULL;
}

/**
 *
 */
static thread_t* help_clicmd(int argc, const char * const * argv, BaseChannel *bchnp){
  (void)bchnp;
  (void)argc;
  (void)argv;

  int32_t i = 0;

  cli_println("Use TAB key for completion, UpArrow for previous command.");
  cli_println("Available commands are:");
  cli_println("-------------------------------------------------------------");

  while(cliutils[i].name != NULL){
    cli_print(cliutils[i].name);
    cli_print(" - ");
    cli_println(cliutils[i].help);
    i++;
  }

  return NULL;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 * Print routine for microrl.
 */
void cli_print(const char *str){
  chnWrite(ShellChnp, (const uint8_t*)str, strlen(str));
}

/**
 * Print routine for microrl.
 */
void cli_print(double var) {
  const size_t N = 33;
  char buf[N];
  memset(buf, 0, sizeof(buf));

  snprintf(buf, N-1, "%f", var);
  cli_print(buf);
}

/**
 * Print routine for microrl.
 */
void cli_print(unsigned int var) {
  const size_t N = 33;
  char buf[N];
  memset(buf, 0, sizeof(buf));

  snprintf(buf, N-1, "%u", var);
  cli_print(buf);
}

/**
 * Print routine for microrl.
 */
void cli_print(int var) {
  const size_t N = 33;
  char buf[N];
  memset(buf, 0, sizeof(buf));

  snprintf(buf, N-1, "%i", var);
  cli_print(buf);
}

/**
 * Convenience function
 */
void cli_println(const char *str){
  cli_print(str);
  cli_print(ENDL);
}

/**
 * Convenience function
 */
void cli_put(const char chr){
  chnWrite(ShellChnp, (const uint8_t*)&chr, 1);
}

/**
 * Read routine
 */
char get_char (void){
  char buf;
  chnRead(ShellChnp, (uint8_t*)&buf, 1);
  return buf;
}

/**
 * helper function
 * Inserts new line symbol if passed string does not contain NULL termination.
 * Must be used in combination with snprintf() function.
 */
void cli_print_long(const char * str, int n, int nres){
  cli_print(str);
  if (nres > n)
    cli_print(ENDL);
}

/**
 *
 */
void KillShellThreads(void) {
  if (NULL != shell_tp) {
    chThdTerminate(shell_tp);
    chThdWait(shell_tp);
    shell_tp = NULL;
  }
}

/**
 *
 */
void SpawnShellThreads(void *bchnp) {

  shell_tp = chThdCreateStatic(ShellThreadWA,
                            sizeof(ShellThreadWA),
                            SHELLPRIO,
                            ShellThread,
                            bchnp);
  if (shell_tp == NULL)
    osalSysHalt("Can not allocate memory");
}

