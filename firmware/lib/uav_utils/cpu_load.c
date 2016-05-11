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

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 * @brief   Estimate CPU usage. Return tens of persents.
 */
#if CH_CFG_USE_REGISTRY && !CH_CFG_NO_IDLE_THREAD && CH_DBG_THREADS_PROFILING

static thread_t *idle_thread = NULL;
static systime_t last_systick = 0;
static systime_t last_idletick = 0;

uint16_t getCpuLoad(void){

  systime_t i, s, i_tmp, s_tmp;

  if (NULL == idle_thread) {
    idle_thread = chRegFindThreadByName("idle");
    chDbgCheck(NULL != idle_thread);
  }

  osalSysLock();
  s_tmp = chVTGetSystemTimeX();
  i_tmp = chThdGetTicksX(idle_thread);
  osalSysUnlock();

  s = s_tmp - last_systick;
  i = i_tmp - last_idletick;

  last_systick = s_tmp;
  last_idletick = i_tmp;

  return ((s - i) * 1000) / s;
}

#else
uint16_t getCpuLoad(void) {
  return -1;
}
#endif
