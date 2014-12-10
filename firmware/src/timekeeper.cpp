#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "pads.h"
#include "global_flags.h"
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

/* delta between RTC and GPS (sec.) */
#define TIME_CORRECTION_THRESHOLD       2

/* Approximated build time stamp. Needed to prevent setting of current
 * time in past. Probably it must be automatically calculated every build. */
#define BUILD_TIME                      1418207643

/* timer autoreload value */
#define RTC_TIMER_STEP                  65535

#define RTC_GPTD                        GPTD6

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern chibios_rt::BinarySemaphore ppstimesync_sem;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */
static void gptcb(GPTDriver *gptp);

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static int64_t UnixUsec;
static int64_t TimeUsGps;

bool TimeKeeper::ready = false;

/*
 * GPT configuration.
 */
static const GPTConfig gptcfg = {
  1000000,    /* 1MHz timer clock.*/
  gptcb,      /* Timer callback.*/
  0,
  0
};

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
static void rtc_set_time_unix_sec(time_t t) {
  RTCDateTime timespec;
  struct tm tim;

  localtime_r(&t, &tim);
  rtcConvertStructTmToDateTime(&tim, 0, &timespec);
  rtcSetTime(&RTCD1, &timespec);
}

/**
 *
 */
static int64_t rtc_get_time_unix_usec(void) {
  RTCDateTime timespec;
  struct tm tim;

  rtcGetTime(&RTCD1, &timespec);
  rtcConvertDateTimeToStructTm(&timespec, &tim);

  return (int64_t)mktime(&tim) * 1000000;
}

/*
 * GPT callback.
 */
static void gptcb(GPTDriver *gptp) {
  (void)gptp;

  osalSysLockFromISR();
  UnixUsec += RTC_TIMER_STEP;
  osalSysUnlockFromISR();
}

/**
 *
 */
static THD_WORKING_AREA(TimekeeperThreadWA, 512) __attribute__((section(".bss.ccm_ram")));
THD_FUNCTION(TimekeeperThread, arg) {
  chRegSetThreadName("Timekeeper");
  TimeKeeper *self = (TimeKeeper *)arg;
//  int64_t  gps_time = 0;
  msg_t sem_status = MSG_RESET;
//  time_t t1;
//  time_t t2;
//  time_t dt;

  while (!chThdShouldTerminateX()) {
    sem_status = ppstimesync_sem.wait(MS2ST(2000));
    if (sem_status == MSG_OK && (1 == GlobalFlags.time_gps_good)){
      osalThreadSleepMilliseconds(100);
      (void)self;
//      chSysLock();
//      gps_time = TimeUsGps;
//      chSysUnlock();
//
//      /* now correct time in internal RTC (if needed) */
//      t1 = gps_time / 1000000;
//      t2 = rtcGetTimeUnixSec(&RTCD1);
//      dt = t1 - t2;
//
//      if (abs(dt) > TIME_CORRECTION_THRESHOLD)
//        rtcSetTimeUnixSec(&RTCD1, t1);
    }
  }

  chThdExit(MSG_OK);
  return MSG_OK;
}

/**
 *
 */
void TimeKeeper::PPS_ISR(void) {

  chSysLockFromISR();
  chSysUnlockFromISR();
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

  UnixUsec = rtc_get_time_unix_usec();

  gptStart(&RTC_GPTD, &gptcfg);
  gptStartContinuous(&RTC_GPTD, RTC_TIMER_STEP);

  worker = chThdCreateStatic(TimekeeperThreadWA, sizeof(TimekeeperThreadWA),
                          TIMEKEEPERPRIO, TimekeeperThread, this);
  osalDbgCheck(nullptr != worker); // Can not allocate memory
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
  worker = nullptr;
}

/**
 *
 */
int64_t TimeKeeper::utc(void) {
  int64_t ret;
  uint32_t cnt1, cnt2;

  osalSysLock();
  cnt1 = RTC_GPTD.tim->CNT;
  ret  = UnixUsec;
  cnt2 = RTC_GPTD.tim->CNT;
  osalSysUnlock();

  if (cnt2 >= cnt1)
    return ret + cnt1;
  else
    return ret + cnt1 + RTC_TIMER_STEP;
}

/**
 * Set current lightweight time.
 */
void TimeKeeper::utc(int64_t t) {

  osalDbgCheck(true == ready);

  osalSysLock();
  UnixUsec = t;
  osalSysUnlock();
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
thread_t* date_clicmd(int argc, const char * const * argv, SerialDriver *sdp) {
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
      rtc_set_time_unix_sec(tv_sec);
      TimeKeeper::utc(tv_usec);
    }
  }

  /* error handler */
  else{
    int n = 40;
    int nres = 0;
    char str[n];

    tv_usec = rtc_get_time_unix_usec();
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
