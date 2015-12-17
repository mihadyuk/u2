#ifndef MAIN_H_
#define MAIN_H_

/* chibios includes */
#ifdef __cplusplus
  #include "ch.hpp"
#else
  #include "ch.h"
#endif

#include "hal.h"

#define __CCM__ __attribute__((section(".ram4")))

/* Heap size for dynamic thread creation */
#define THREAD_HEAP_SIZE    (1024 * 4)

///* Disarm halting on panic and changing it to soft reset after this amount of time */
//#define HALT_DISARM_TMO_SEC             30

/*
 * Size of mavlink send buffer. It must be big enogh to store
 * (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) bytes.
 *
 * WARNING! Buffer must be aligned to CPU word boundaries. In other
 * words its size must be multiple of 4 on Cortex-Mx
 */
#define MAVLINK_SENDBUF_SIZE      264

/*
 * thread priorities
 */
#define ADISPRIO          (NORMALPRIO)
#define BAROMETERPRIO     (NORMALPRIO + 2)
#define CMDPRIO           (NORMALPRIO - 2)
#define MISSIONRECVRPRIO  (NORMALPRIO)
#define GPSPRIO           (NORMALPRIO + 1)
#define MAVPOSTMANPRIO    (NORMALPRIO - 1)
#define TELEMETRYDISPPRIO (NORMALPRIO + 5) // it must be high because it just schedule sending
#define MPU6050PRIO       (ADISPRIO + 1)
#define TIMEKEEPERPRIO    (NORMALPRIO - 1)
#define SHELLPRIO         (NORMALPRIO - 10)

/*
 * timestamp for mavlink messages
 */
#if (CH_CFG_ST_FREQUENCY) >= 1000
#define TIME_BOOT_MS ((chVTGetSystemTimeX()) / ((CH_CFG_ST_FREQUENCY) / 1000))
#endif

#define ForbiddenDestructor() osalSysHalt("Destruction forbidden.")

#endif /* MAIN_H_ */
