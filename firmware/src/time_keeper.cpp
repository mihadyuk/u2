#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "pads.h"
#include "global_flags.h"
#include "time_keeper.hpp"
#include "exti_local.hpp"
#include "eb500.hpp"

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
#define TIME_CORRECTION_THRESHOLD       4

/* Approximated build time stamp. Needed to prevent setting of current
 * time in past. Probably it must be automatically calculated every build. */
#define BUILD_TIME                      1418207643

/* timer autoreload value */
#define RTC_TIMER_STEP                  UINT16_MAX

/* Very dirty oscillator correction. Please remove it */
#define RTC_TIMER_SKEW                  -3

#define RTC_GPTD                        GPTD6

/**
 * structure for PPS jitter statistic
 */
struct time_staticstic_t {
  int32_t   max = 0;            /**< @brief Minimal measurement.    */
  int32_t   min = 0;            /**< @brief Maximum measurement.    */
  int32_t   last = 0;           /**< @brief Last measurement.       */
  size_t    n = 0;              /**< @brief Number of measurements. */
  int32_t   cumulative = 0;     /**< @brief Cumulative measurement. */
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

static int64_t pps_sample_us_prev = TIME_INVALID;
static int64_t pps_sample_us = TIME_INVALID;

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

static chibios_rt::BinarySemaphore ppstimesync_sem(true);

static gps::gps_data_t gps_data __CCM__;

static time_staticstic_t time_stat __CCM__;

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
  unix_usec += RTC_TIMER_STEP + RTC_TIMER_SKEW;
  osalSysUnlockFromISR();
}

/**
 *
 */
static void jitter_stats_update(void) {

  int32_t speed_delta = (pps_sample_us - pps_sample_us_prev) - 1000000;

  time_stat.n++;
  time_stat.cumulative += speed_delta;
  time_stat.last = speed_delta;
  if (speed_delta < time_stat.min)
    time_stat.min = speed_delta;
  else if (speed_delta > time_stat.max)
    time_stat.max = speed_delta;
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
static THD_WORKING_AREA(TimekeeperThreadWA, 512) __CCM__;
THD_FUNCTION(TimekeeperThread, arg) {
  chRegSetThreadName("Timekeeper");
  TimeKeeper *self = static_cast<TimeKeeper *>(arg);
  (void)self;
  msg_t sem_status = MSG_RESET;
  chibios_rt::EvtListener el;
  event_gps.registerMask(&el, EVMSK_GPS_UPATED);
  eventmask_t gps_evt;

  while (!chThdShouldTerminateX()) {
    sem_status = ppstimesync_sem.wait(MS2ST(1200));
    if (MSG_OK == sem_status) {
      el.getAndClearFlags();

      jitter_stats_update();

      while (true) { /* wait first measurement with rounde seconds */
        gps_evt = chEvtWaitOneTimeout(EVMSK_GPS_UPATED, MS2ST(1200));
        if (EVMSK_GPS_UPATED == gps_evt) {
          GPSGet(gps_data);
          if (gps_data.fix_valid && gps_data.sec_round) {
            int64_t tmp = 1000000;
            tmp *= mktime(&gps_data.time);
            osalSysLock();
            time_gps_us = tmp;
            osalSysUnlock();

            /* now correct time in internal RTC (if needed) */
            int32_t t1 = time_gps_us / 1000000;
            int32_t t2 = rtc_get_time_unix(nullptr);
            int32_t dt = t1 - t2;

            if (abs(dt) > TIME_CORRECTION_THRESHOLD)
              rtc_set_time(&gps_data.time);

            break; // while(true)
          }
        }
      }
    }
  }

  event_gps.unregister(&el);
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
TimeKeeper::TimeKeeper(void) {
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
    return ret + cnt1 + RTC_TIMER_STEP + RTC_TIMER_SKEW;
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

  pps_sample_us_prev = pps_sample_us;
  pps_sample_us = utc();
  ppstimesync_sem.signalI();
}

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
      rtc_set_time_unix(tv_sec);
      TimeKeeper::utc(tv_usec);
    }
  }

  /* error handler */
  else{
    const int n = 40;
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
    tv_usec = time_gps_us;
    chSysUnlock();
    nres = snprintf(str, n, "%lld", tv_usec);
    cli_print("GPS: ");
    cli_print_long(str, n, nres);
    cli_print(" uS since Unix epoch. ");
    TimeKeeper::format_time(tv_usec, str, n);
    cli_println(str);

    cli_println("PPS jitter:");
    nres = snprintf(str, n, "%ld", time_stat.last);
    cli_print("last:       ");
    cli_print_long(str, n, nres);
    nres = snprintf(str, n, "%ld", time_stat.min);
    cli_print("min:        ");
    cli_print_long(str, n, nres);
    nres = snprintf(str, n, "%ld", time_stat.max);
    cli_print("max:        ");
    cli_print_long(str, n, nres);
    nres = snprintf(str, n, "%ld", time_stat.cumulative);
    cli_print("cumulative: ");
    cli_print_long(str, n, nres);
    nres = snprintf(str, n, "%u", time_stat.n);
    cli_print("iterations: ");
    cli_print_long(str, n, nres);

    cli_println("");
    cli_println("To adjust time run 'date N'");
    cli_println("    where 'N' is count of seconds (UTC) since Unix epoch.");
    cli_println("    you can obtain this value from Unix command line: 'date -u +%s'");
  }

  /* stub */
  return nullptr;
}
