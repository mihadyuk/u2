#include <string.h>
#include <time.h>

#include "main.h"
#include "pads.h"
#include "chprintf.h"

#include "mav_logger.hpp"
#include "mav_codec.h"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define SDC_POLLING_INTERVAL            MS2ST(200)
#define SDC_POLLING_DELAY               5
#define SDC_POWER_TIMEOUT               MS2ST(400)
#define SYNC_PERIOD                     MS2ST(5000)

/* buffer size for file name string */
#define MAX_FILENAME_SIZE               32

/*
 * Проверяет вёрнутый статус.
 */
#define err_check()   {if(err != FR_OK){exit(MSG_RESET);}}

#define MAVLINK_LOG_FORMAT              FALSE

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

/* SDIO configuration. */
static SDCConfig sdccfg = {
    NULL,
    SDC_MODE_4BIT
};

/* FS object.*/
static FATFS SDC_FS;

/* FS mounted and ready.*/
static bool fs_ready = false;

/* флаг необходимости синхронизации по таймауту */
static bool sync_tmo = false;

/* флаг обозначающий наличие свежих данных, чтобы зря не дергать sync */
static bool fresh_data = false;

/* по этому таймеру будет синхронизироваться файл лога */
static virtual_timer_t sync_vt;

__CCM__ static unpacked_sdc_cid_t cidsdc;
__CCM__ static unpacked_sdc_csd_20_t csd20;
__CCM__ static unpacked_sdc_csd_10_t csd10;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */
/*
 * SD card insertion event.
 */
static void insert_handler(void) {
  FRESULT err;

  sdcStart(&SDCD1, &sdccfg);
  microsd_power_on();
  osalThreadSleep(SDC_POWER_TIMEOUT);

  /* On insertion SDC initialization and FS mount. */
  if (HAL_FAILED == sdcConnect(&SDCD1))
    return;
  else {
    _mmcsd_unpack_sdc_cid((MMCSDBlockDevice *)&SDCD1, &cidsdc);
    _mmcsd_unpack_csd_v20((MMCSDBlockDevice *)&SDCD1, &csd20);
    _mmcsd_unpack_csd_v10((MMCSDBlockDevice *)&SDCD1, &csd10);
  }

  err = f_mount(&SDC_FS, "/", 1);
  if (err != FR_OK) {
    sdcDisconnect(&SDCD1);
    sdcStop(&SDCD1);
    fs_ready = false;
    return;
  }

  fs_ready = true;
}

/*
 * SD card removal event.
 */
static void remove_handler(void) {

  if (fs_ready == false) {
    f_mount(NULL, 0, 0);
    fs_ready = false;
  }

  if ((&SDCD1)->state == BLK_READY) {
    sdcDisconnect(&SDCD1);
    sdcStop(&SDCD1);
  }

  fs_ready = false;
}

/**
 * Create human readable name for file.
 */
static size_t name_from_time(char *buf) {
  struct tm tim;
  RTCDateTime timespec;

  rtcGetTime(&RTCD1, &timespec);
  rtcConvertDateTimeToStructTm(&timespec, &tim, nullptr);

#if MAVLINK_LOG_FORMAT
  return strftime(buf, MAX_FILENAME_SIZE, "%F_%H.%M.%S.mavlink", &tim);
#else
  return strftime(buf, MAX_FILENAME_SIZE, "%F_%H.%M.%S.raw", &tim);
#endif
}

/**
 *
 */
static void sync_cb(void *par) {
  (void)par;
  osalSysLockFromISR();
  chVTSetI(&sync_vt, SYNC_PERIOD, &sync_cb, NULL);
  sync_tmo = true;
  osalSysUnlockFromISR();
}

/**
 * По нескольким критериям определяет, надо ли сбрасывать буфер и при
 * необходимости сбрасывает.
 */
static FRESULT fs_sync(FIL *Log) {
  FRESULT err = FR_OK;

  if (sync_tmo && fresh_data) {
    err = f_sync(Log);
    sync_tmo = false;
    fresh_data = false;
  }
  return err;
}

/**
 * Get id of data
 * Pack it
 * Store to FS buffer
 * Raise bool flag if fresh data available
 */
FRESULT MavLogger::append_log(mavMail *mail, bool *fresh_data) {
  UINT bytes_written;
  uint8_t *fs_buf;
  FRESULT err = FR_OK;
  size_t len;
  mavlink_message_t log_msg;
  uint8_t recordbuf[MAVLINK_SENDBUF_SIZE + sizeof(uint64_t)];

  /**/
  mavlink_encode(mail->msgid, mail->compid, &log_msg, mail->mavmsg);
  mail->release();

  memset(recordbuf, 0, sizeof(recordbuf));
#if MAVLINK_LOG_FORMAT
  uint64_t timestamp = 0;
  len = mavlink_msg_to_send_buffer(recordbuf + sizeof(timestamp), &log_msg);
  memcpy(recordbuf, &timestamp, sizeof(timestamp));
  fs_buf = double_buf.append(recordbuf, len + sizeof(timestamp));
#else /* MAVLINK_LOG_FORMAT */
  len = mavlink_msg_to_send_buffer(recordbuf, &log_msg);
  fs_buf = double_buf.append(recordbuf, len);
#endif /* MAVLINK_LOG_FORMAT */

  /* write data to file */
  if (fs_buf != nullptr) {
    err = f_write(&log_file, fs_buf, double_buf.size(), &bytes_written);
    if (err == FR_OK)
      *fresh_data = true;
  }

  return err;
}

/**
 * Если произошла ошибка - поток логгера просто тушится,
 * потому что исправить всё равно ничего нельзя.
 */
void MavLogger::main(void) {
  setName("MicroSD");

  FRESULT err;
  uint32_t clusters;
  FATFS *fsp;
  mavMail *mail;

  chVTObjectInit(&sync_vt);
  chVTSet(&sync_vt, SYNC_PERIOD, &sync_cb, NULL);

  /* wait card readyness */
NOT_READY:
  while (!sdcIsCardInserted(&SDCD1)) {
    if (this->shouldTerminate())
      goto EXIT;
    osalThreadSleep(SDC_POLLING_INTERVAL);
  }
  osalThreadSleep(SDC_POLLING_INTERVAL);
  if (!sdcIsCardInserted(&SDCD1))
    goto NOT_READY;
  else
    insert_handler();

  /* fs mounted? */
  if (!fs_ready)
    goto NOT_READY;

  /* are we have at least 16MB of free space? */
  err = f_getfree("/", &clusters, &fsp);
  err_check();
  if ((clusters * (uint32_t)SDC_FS.csize * (uint32_t)MMCSD_BLOCK_SIZE) < (16*1024*1024))
    goto EXIT;

  /* open file for writing log */
  char namebuf[MAX_FILENAME_SIZE];
  name_from_time(namebuf);
  err = f_open(&log_file, namebuf, FA_WRITE | FA_CREATE_ALWAYS);
  err_check();
  err = f_sync(&log_file);
  err_check();

  ready = true;
  while (!this->shouldTerminate()) {
    if (!sdcIsCardInserted(&SDCD1)) {
      ready = false;
      remove_handler();
      osalThreadSleepMilliseconds(100);
      osalSysLock();
      size_t used = logwriter_mb.getUsedCountI();
      osalSysUnlock();
      while (used--) {
        logwriter_mb.fetch(&mail, MS2ST(100));
        mail->release();
      }
      goto NOT_READY;
    }

    /* */
    if (logwriter_mb.fetch(&mail, MS2ST(100)) == MSG_OK) {
      err = append_log(mail, &fresh_data);
      err_check();
    }

    err = fs_sync(&log_file);
    err_check();
  }

EXIT:
  ready = false;
  exit(MSG_OK);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 * @note    just drop message if logger is not ready
 */
msg_t MavLogger::write(mavMail* mail) {
  msg_t ret = MSG_RESET;

  if (true == ready) {
    ret = logwriter_mb.post(mail, TIME_IMMEDIATE);
    if (MSG_OK != ret) {
      mail->release();
      this->drop_cnt++;
    }
  }
  else {
    mail->release();
  }

  return ret;
}

/**
 *
 */
void MavLogger::stop(void) {
  this->requestTerminate();
  this->wait();
  microsd_power_off();
}
