#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "pads.h"
#include "global_flags.h"
#include "time_keeper.hpp"
#include "exti_local.hpp"
#include "ublox.hpp"

/*
 * Время работает следующим образом:
 * - при старте читается значение из RTC
 * - это значение является отправной точкой для рассчета текущего времени
 *   на основе значения таймера
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
#define RTC_TIMER_STEP                  UINT16_MAX

/* Very dirty oscillator correction. Please remove it */
#define RTC_TIMER_SKEW_DEFAULT          -3

#define RTC_GPTD                        GPTD6

/**
 *
 */
struct drift_estimate_t {
  int64_t sample = TIME_INVALID;
  int64_t sample_prev = TIME_INVALID;
};

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
static void gptcb(GPTDriver *gptp);

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static int64_t unix_usec = TIME_INVALID;
static int64_t time_gps_us = TIME_INVALID;

static int64_t timer_skew = RTC_TIMER_SKEW_DEFAULT;

static drift_estimate_t drift_est;

bool TimeKeeper::ready = false;
bool TimeKeeper::time_verified = false;

/*
 * GPT configuration.
 */
static const GPTConfig gptcfg = {
  1000000,    /* 1MHz timer clock.*/
  gptcb,      /* Timer callback.*/
  0,
  0
};

static chibios_rt::BinarySemaphore ppstimesync_sem(true);

__CCM__ static gnss::gnss_data_t gps;

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
/*
 * GPT callback.
 */
static void gptcb(GPTDriver *gptp) {
  (void)gptp;

  osalSysLockFromISR();
  unix_usec += RTC_TIMER_STEP + timer_skew;
  osalSysUnlockFromISR();
}

/**
 *
 */
static void rtc_set_time(struct tm *tim) {
  RTCDateTime timespec;
  rtcConvertStructTmToDateTime(tim, 0, &timespec);
  rtcSetTime(&RTCD1, &timespec);
}

/**
 *
 */
static void rtc_set_time_unix(time_t t) {
  RTCDateTime timespec;
  struct tm tim;

  localtime_r(&t, &tim);
  rtcConvertStructTmToDateTime(&tim, 0, &timespec);
  rtcSetTime(&RTCD1, &timespec);
}

/**
 *
 */
static time_t rtc_get_time_unix(uint32_t *tv_msec) {
  RTCDateTime timespec;
  struct tm tim;

  rtcGetTime(&RTCD1, &timespec);
  rtcConvertDateTimeToStructTm(&timespec, &tim, tv_msec);

  return mktime(&tim);
}

/**
 *
 */
static int64_t rtc_get_time_unix_usec(void) {
  uint32_t tv_msec;
  int64_t ret;

  ret = (int64_t)rtc_get_time_unix(&tv_msec) * (int64_t)1000000;
  ret += tv_msec * 1000;
  return ret;
}

/**
 *
 */
THD_FUNCTION(TimeKeeper::TimekeeperThread, arg) {
  chRegSetThreadName("Timekeeper");
  TimeKeeper *self = static_cast<TimeKeeper *>(arg);

  self->GNSS.subscribe(&gps);

  while (!chThdShouldTerminateX()) {
    if ((gps.fresh) && (gps.fix > 0) && (0 == gps.msec)) {
      int64_t tmp = 1000000;
      tmp *= mktime(&gps.time);

      osalSysLock();
      time_gps_us = tmp;
      if (! time_verified) {
        time_verified = true;
        unix_usec = time_gps_us;
      }
      osalSysUnlock();

      /* now correct time in internal RTC (if needed) */
      int32_t t1 = time_gps_us / 1000000;
      int32_t t2 = rtc_get_time_unix(nullptr);
      int32_t dt = t1 - t2;

      if (abs(dt) > TIME_CORRECTION_THRESHOLD)
        rtc_set_time(&gps.time);
    }

    if (gps.fresh) {
      gps.fresh = false;
    }

    osalThreadSleepMilliseconds(20);
  }

  self->GNSS.unsubscribe(&gps);
  chThdExit(MSG_OK);
}



/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
TimeKeeper::TimeKeeper(gnss::GNSSReceiver &GNSS) : GNSS(GNSS) {
  ready = false;
}

/**
 *
 */
void TimeKeeper::start(void) {

  unix_usec = rtc_get_time_unix_usec();

  gptStart(&RTC_GPTD, &gptcfg);
  gptStartContinuous(&RTC_GPTD, RTC_TIMER_STEP);

  worker = chThdCreateStatic(TimekeeperThreadWA, sizeof(TimekeeperThreadWA),
                             TIMEKEEPERPRIO, TimekeeperThread, this);
  osalDbgCheck(nullptr != worker); // Can not allocate memory
  Exti.pps(true);
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
  Exti.pps(false);
}

/**
 *
 */
int64_t TimeKeeper::utc(void) {
  int64_t ret;
  uint32_t cnt1, cnt2;
  syssts_t sts;

  sts = osalSysGetStatusAndLockX();
  cnt1 = RTC_GPTD.tim->CNT;
  ret  = unix_usec;
  cnt2 = RTC_GPTD.tim->CNT;
  osalSysRestoreStatusX(sts);

  if (cnt2 >= cnt1)
    return ret + cnt1;
  else
    return ret + cnt1 + RTC_TIMER_STEP + timer_skew;
}

/**
 * Set current lightweight time.
 */
void TimeKeeper::utc(int64_t t) {

  osalDbgCheck(true == ready);

  osalSysLock();
  unix_usec = t;
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

/**
 *
 */
void TimeKeeper::PPS_ISR_I(void) {

  drift_est.sample_prev = drift_est.sample;
  drift_est.sample = utc();
  ppstimesync_sem.signalI();
}

/**
 *
 */
static size_t format_usec(char *buf, size_t N, int64_t tv_usec) {
  int32_t i = tv_usec / 1000000;
  int32_t f = tv_usec % 1000000;

  if (0 != f)
    return snprintf(buf, N, "%ld.%ld", i, f);
  else
    return snprintf(buf, N, "%ld.000000", i);
}

/**
 * Command to handle RTC.
 */
thread_t* date_clicmd(int argc, const char * const * argv, BaseChannel *bchnp) {
  (void)bchnp;

  time_t tv_sec = 0;
  int64_t tv_usec = 0;
  int sscanf_status;

  /**/
  if (argc == 2) {
    if (0 == strcmp(argv[0], "set")) {
      sscanf_status = sscanf(argv[1], "%i", (int*)&tv_sec);
      if (sscanf_status != 1)
        cli_println("ERROR. Time value inconsistent");
      else if (tv_sec < BUILD_TIME)
        cli_println("ERROR. Time in past");
      else{
        tv_usec = tv_sec;
        tv_usec *= 1000000;
        rtc_set_time_unix(tv_sec);
        TimeKeeper::utc(tv_usec);
      }
    }
    else if (0 == strcmp(argv[0], "drift")) {
      sscanf_status = sscanf(argv[1], "%i", (int*)&tv_sec);
      if (sscanf_status != 1)
        cli_println("ERROR. Value inconsistent");
      else {
        timer_skew = tv_sec;
      }
    }
  }

  /* 1 argument */
  else if (argc == 1) {
    if (0 == strcmp(argv[0], "drift")) {
      int tmp = timer_skew;
      cli_print(tmp);
      cli_println("");
    }
  }

  /* error handler */
  else {
    const size_t n = 40;
    size_t nres = 0;
    char str[n];

    tv_usec = rtc_get_time_unix_usec();
    nres = format_usec(str, n, tv_usec);
    cli_print("RTC: ");
    cli_print_long(str, n, nres);
    cli_print(" S since Unix epoch. ");
    TimeKeeper::format_time(tv_usec, str, n);
    cli_println(str);

    tv_usec = TimeKeeper::utc();
    nres = format_usec(str, n, tv_usec);
    cli_print("CNT: ");
    cli_print_long(str, n, nres);
    cli_print(" S since Unix epoch. ");
    TimeKeeper::format_time(tv_usec, str, n);
    cli_println(str);

    chSysLock();
    tv_usec = time_gps_us;
    chSysUnlock();
    nres = format_usec(str, n, tv_usec);
    cli_print("GPS: ");
    cli_print_long(str, n, nres);
    cli_print(" S since Unix epoch. ");
    TimeKeeper::format_time(tv_usec, str, n);
    cli_println(str);

    nres = snprintf(str, n, "Drift estimate: %lld", drift_est.sample - drift_est.sample_prev - 1000000);
    cli_print_long(str, n, nres);

    cli_println("");
    cli_println("To adjust time run 'date set N'");
    cli_println("    where 'N' is count of seconds (UTC) since Unix epoch.");
    cli_println("    you can obtain this value from Unix command line: 'date -u +%s'");
  }

  /* stub */
  return nullptr;
}
