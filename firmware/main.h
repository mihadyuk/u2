#ifndef MAIN_H_
#define MAIN_H_

/* chibios includes */
#ifdef __cplusplus
  #include "ch.hpp"
#else
  #include "ch.h"
#endif

#include "hal.h"

/* Heap size for dynamic thread creation */
#define THREAD_HEAP_SIZE    (1024 * 4)

//
/* Enable messages about unhandled mavlink messages received */
#define MAVLINK_UNHANDLED_MSG_DEBUG     FALSE

///* Disarm halting on panic and changing it to soft reset after this amount of time */
//#define HALT_DISARM_TMO_SEC             30
//
///* use hardcoded value instead of measured */
//#define GYRO_UPDATE_PERIOD_HARDCODED    TRUE

/*
 * Size of mavlink send buffer. It must be big enogh to store
 * (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) bytes.
 *
 * WARNING! Buffer must be aligned to CPU word boundaries. In other
 * words its size must be multiple of 4 on Cortex-Mx
 */
#define MAVLINK_SENDBUF_SIZE      264

/* приоритеты для потоков */
#define I2CPRIO           (NORMALPRIO - 5)
#define TIMEKEEPERPRIO    (I2CPRIO - 1)
#define LINKPRIO          (NORMALPRIO - 5)
#define CONTROLLERPRIO    (NORMALPRIO)
#define CMDPRIO           (LINKPRIO - 2)
#define GPSPRIO           (NORMALPRIO - 2)
#define TELEMTRYPRIO      (LINKPRIO - 1)
#define SHELLPRIO         (NORMALPRIO - 10)

///* константы для мавлинка */
//#define GROUND_STATION_ID   255
//
//
///* метка времени для пакетов */
//#if (CH_FREQUENCY) >= 1000
//#define TIME_BOOT_MS ((chTimeNow()) / ((CH_FREQUENCY) / 1000))
//#endif
//
///* stop watchdog timer in debugging mode */
///*unlock PR register*/
///*set 1.6384s timeout*/
///*start watchdog*/
/*
#define WATCHDOG_INIT {\
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;\
    IWDG->KR = 0x5555;\
    IWDG->PR = 16;\
    IWDG->KR = 0xCCCC;}

#define WATCHDOG_RELOAD {IWDG->KR = 0xAAAA;}
*/

#endif /* MAIN_H_ */
