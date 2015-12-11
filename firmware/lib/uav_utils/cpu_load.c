#include <string.h>
#include "main.h"

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

#if CH_CFG_USE_REGISTRY && !CH_CFG_NO_IDLE_THREAD && CH_DBG_THREADS_PROFILING

static thread_t *idle_thread = NULL;
static systime_t last_systick = 0;
static systime_t last_idletick = 0;

static thread_t *get_idle_thread(void){

  thread_t *first = chRegFirstThread();
  const char *name;

  while (NULL != first) {
    name = chRegGetThreadNameX(first);
    if (0 == strcmp("idle", name))
      return first;
    first = chRegNextThread(first);
  }
  return NULL; // nothing found
}
#endif

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 * @brief   Estimate CPU usage. Return tens of persents.
 */
#if CH_CFG_USE_REGISTRY && !CH_CFG_NO_IDLE_THREAD && CH_DBG_THREADS_PROFILING

uint16_t getCpuLoad(void){

  systime_t i, s;

  if (NULL == idle_thread) {
    idle_thread = get_idle_thread();
    chDbgCheck(NULL != idle_thread);
  }

  s = chVTGetSystemTimeX() - last_systick;
  i = chThdGetTicksX(idle_thread) - last_idletick;

  last_systick = chVTGetSystemTimeX();
  last_idletick = chThdGetTicksX(idle_thread);

  return ((s - i) * 1000) / s;
}

#else
uint16_t getCpuLoad(void){
  return 0;
}
#endif
