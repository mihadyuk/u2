#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "pads.h"
#include "chrtclib.h"
#include "global_flags.h"

#include "exti_local.hpp"
#include "timekeeper.hpp"

/*
 * Время работает следующим образом:
 * - при старте читается значение из RTC
 * - это значение является отправной точкой для рассчета текущего времени
 *   на основе значения системного таймера
 * - вычитанное значение никогда не корректируется во избежание геморроя
 *   с логами
 * - время в ячейке RTC корректируется при наличии спутникового времени,
 *   чтобы при следующем запуске устройства загрузить это значение в
 *   качестве точки отсчета
 * - сама точка отсчета по GPS _не_ корректируется _никогда_
 *
 * Иными словами, для получения адекватной метки времени в логах после
 * замены батарейки или длительного хранения надо:
 * - обеспечить устойчивый прием сигналов GPS
 * - дождаться 3-мерного нав. решения
 * - перезагрузить устройство (можно с помощью выкл\вкл)
 *
 * Хитрые форсированные и прочие ручные методы установки времени будут сделаны
 * когда-нибудь в следующей жизни.
 */

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define SOFT_PPS_LEN                    MS2ST(10)

/* delta between RTC and GPS (sec.) */
#define TIME_CORRECTION_THRESHOLD       1

/* Approximated build time stamp. Needed to prevent setting of current
 * time in past. Probably it must be automatically calculated every build. */
#define BUILD_TIME                      1382986933

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern const int64_t TimeUsGps;
extern chibios_rt::BinarySemaphore ppstimesync_sem;
extern chibios_rt::BinarySemaphore ppsgps_sem;
extern chibios_rt::BinarySemaphore ppsstanag_sem;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/* Boot timestamp (microseconds since UNIX epoch). Inits in boot time. Latter
 * uses to calculate current time using current systick value. */
static int64_t BootTimestamp = 0;

bool TimeKeeper::ready = false;

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
static WORKING_AREA(TimekeeperThreadWA, 512) __attribute__((section(".bss.ccm_ram")));
static msg_t TimekeeperThread(void *arg){
  chRegSetThreadName("Timekeeper");
  (void)arg;

  int64_t  gps_time = 0;
  msg_t sem_status = RDY_RESET;
  time_t t1;
  time_t t2;
  time_t dt;

  while (!chThdShouldTerminate()) {
    sem_status = ppstimesync_sem.waitTimeout(MS2ST(2000));
    if (sem_status == RDY_OK && (1 == GlobalFlags.time_gps_good)){
      chSysLock();
      gps_time = TimeUsGps;
      chSysUnlock();

      /* now correct time in internal RTC (if needed) */
      t1 = gps_time / 1000000;
      t2 = rtcGetTimeUnixSec(&RTCD1);
      dt = t1 - t2;

      if (abs(dt) > TIME_CORRECTION_THRESHOLD)
        rtcSetTimeUnixSec(&RTCD1, t1);
    }
  }

  chThdExit(0);
  return 0;
}

/**
 *
 */
#if defined(BOARD_NTLAB_GRIFFON_NS)
static WORKING_AREA(PpsThreadWA, 64);
static msg_t PpsThread(void *arg){
  (void)arg;
  chRegSetThreadName("SoftPps");
  systime_t tmp;

  while (!chThdShouldTerminate()){
    tmp = chTimeNow();
    pps_led_on();
    chThdSleep(SOFT_PPS_LEN);
    pps_led_off();
    chThdSleepUntil(tmp + CH_FREQUENCY);
  }

  chThdExit(0);
  return 0;
}
#endif /* defined(BOARD_NTLAB_GRIFFON_NS) */

/**
 *
 */
void TimeKeeper::normal_pps_isr(EXTDriver *extp, expchannel_t channel){
  (void)extp;
  (void)channel;

  chSysLockFromIsr();
  ppsgps_sem.signalI();
  ppstimesync_sem.signalI();
  #if defined(BOARD_NTLAB_GRIFFON_ACS)
  ppsstanag_sem.signalI();
  #endif
  chSysUnlockFromIsr();
}

/**
 *
 */
TimeKeeper::TimeKeeper(void) {
  ready = false;
}

/**
 *
 */
void TimeKeeper::start(void) {

  BootTimestamp = rtcGetTimeUnixUsec(&RTCD1);

  /* RTC reading may be performed after some unpredictable time since boot,
   * so we need to adjust it */
  BootTimestamp -= (int64_t)TIME_BOOT_MS * CH_FREQUENCY;

  worker = chThdCreateStatic(TimekeeperThreadWA,
                          sizeof(TimekeeperThreadWA),
                          TIMEKEEPERPRIO,
                          TimekeeperThread,
                          NULL);
  chDbgCheck(NULL != worker, "Can not allocate memory");

#if defined(BOARD_NTLAB_GRIFFON_NS)
  soft_pps = chThdCreateStatic(PpsThreadWA,
                          sizeof(PpsThreadWA),
                          PPSPRIO,
                          PpsThread,
                          NULL);
  chDbgCheck(NULL != soft_pps, "Can not allocate memory");
#endif /* defined(BOARD_NTLAB_GRIFFON_NS) */

  Exti.Pps(true);
  ready = true;
}

/**
 *
 */
void TimeKeeper::stop(void) {
  ready = false;

  chThdTerminate(worker);
  ppstimesync_sem.signal(); /* speed up termination */
  chThdWait(worker);

  #if defined(BOARD_NTLAB_GRIFFON_NS)
  chThdTerminate(soft_pps);
  chThdWait(soft_pps);
  #endif /* defined(BOARD_NTLAB_GRIFFON_NS) */

  pps_led_off();          // make clean
}

/**
 * Return current time using lightweight approximation to avoid calling
 * of heavy time conversion (from hardware RTC) functions.
 */
int64_t TimeKeeper::utc(void) {

  chDbgCheck(true == ready, "Not ready");
  return BootTimestamp + (int64_t)(TIME_BOOT_MS) * CH_FREQUENCY;
}

/**
 * Set current lightweight time.
 */
void TimeKeeper::utc(int64_t t) {

  chDbgCheck(true == ready, "Not ready");

  chSysLock();
  BootTimestamp = t - (int64_t)(TIME_BOOT_MS) * CH_FREQUENCY;
  chSysUnlock();
}

/**
 *
 */
int TimeKeeper::format_time(char *str, size_t len){
  int64_t time = this->utc();
  return this->format_time(time, str, len);
}

/**
 *
 */
int TimeKeeper::format_time(int64_t time, char *str, size_t len){
  time_t tv_sec;
  uint32_t usec;
  struct tm timp;
  size_t used;

  tv_sec  = time / 1000000;
  usec = time % 1000000;
  localtime_r(&tv_sec, &timp);
  used = strftime(str, len, "%F %H:%M:%S", &timp);
  return snprintf(str+used, len-used, ".%06lu", usec);
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 * Command to handle RTC.
 */
Thread* date_clicmd(int argc, const char * const * argv, SerialDriver *sdp){
  (void)sdp;

  time_t tv_sec = 0;
  int64_t tv_usec = 0;
  int sscanf_status;

  /* 1 argument */
  if (argc == 1){
    sscanf_status = sscanf(argv[0], "%i", (int*)&tv_sec);
    if (sscanf_status != 1)
      cli_println("ERROR. Time value inconsistent");
    else if (tv_sec < BUILD_TIME)
      cli_println("ERROR. Time in past");
    else{
      tv_usec = tv_sec;
      tv_usec *= 1000000;
      rtcSetTimeUnixSec(&RTCD1, tv_sec);
      TimeKeeper::utc(tv_usec);
    }
  }

  /* error handler */
  else{
    int n = 40;
    int nres = 0;
    char str[n];

    tv_usec = rtcGetTimeUnixUsec(&RTCD1);
    nres = snprintf(str, n, "%lld", tv_usec);
    cli_print("RTC: ");
    cli_print_long(str, n, nres);
    cli_print(" uS since Unix epoch. ");
    TimeKeeper::format_time(tv_usec, str, n);
    cli_println(str);

    tv_usec = TimeKeeper::utc();
    nres = snprintf(str, n, "%lld", tv_usec);
    cli_print("CNT: ");
    cli_print_long(str, n, nres);
    cli_print(" uS since Unix epoch. ");
    TimeKeeper::format_time(tv_usec, str, n);
    cli_println(str);

    chSysLock();
    tv_usec = TimeUsGps;
    chSysUnlock();
    nres = snprintf(str, n, "%lld", tv_usec);
    cli_print("GPS: ");
    cli_print_long(str, n, nres);
    cli_print(" uS since Unix epoch. ");
    TimeKeeper::format_time(tv_usec, str, n);
    cli_println(str);

    cli_println("");
    cli_println("To adjust time run 'date N'");
    cli_println("    where 'N' is count of seconds (UTC) since Unix epoch.");
    cli_println("    you can obtain this value from Unix command line: 'date -u +%s'");
  }

  /* stub */
  return NULL;
}
