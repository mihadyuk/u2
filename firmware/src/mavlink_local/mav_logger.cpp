#include <string.h>
#include <time.h>

#include "main.h"
#include "chprintf.h"

#include "mav_logger.hpp"
#include "mav_encode.h"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define SDC_POLLING_INTERVAL            100
#define SDC_POLLING_DELAY               5
#define SYNC_PERIOD                     MS2ST(5000)

/* buffer size for file name string */
#define MAX_FILENAME_SIZE               32

/*
 * Проверяет вёрнутый статус.
 */
#define err_check()   {if(err != FR_OK){return MSG_RESET;}}

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
static const SDCConfig sdccfg = {
  0
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
  /*
   * On insertion SDC initialization and FS mount.
   */
  if (sdcConnect(&SDCD1))
    return;

  err = f_mount(&SDC_FS, 0, 0);
  if (err != FR_OK) {
    sdcDisconnect(&SDCD1);
    sdcStop(&SDCD1);
    return;
  }
  fs_ready = TRUE;
}

/*
 * SD card removal event.
 */
static void remove_handler(void) {

  if (fs_ready == TRUE){
    f_mount(NULL, 0, 0);
    fs_ready = FALSE;
  }

  if ((&SDCD1)->state == BLK_ACTIVE){
    sdcDisconnect(&SDCD1);
    sdcStop(&SDCD1);
  }
  fs_ready = FALSE;
}

/**
 * Create human readable name for file.
 */
static size_t name_from_time(char *buf) {
  struct tm tim;
  RTCDateTime timespec;

  rtcGetTime(&RTCD1, &timespec);
  rtcConvertDateTimeToStructTm(&timespec, &tim);

#if MAVLINK_LOG_FORMAT
  return strftime(buf, MAX_FILENAME_SIZE, "%F_%H.%M.%S.mavlink", &tim);
#else
  return strftime(buf, MAX_FILENAME_SIZE, "%F_%H.%M.%S.raw", &tim);
#endif
}

/**
 *
 */
static void sync_cb(void *par){
  (void)par;
  osalSysLockFromISR();
  chVTSetI(&sync_vt, SYNC_PERIOD, &sync_cb, NULL);
  sync_tmo = TRUE;
  osalSysUnlockFromISR();
}

/**
 * По нескольким критериям определяет, надо ли сбрасывать буфер и при
 * необходимости сбрасывает.
 */
static FRESULT fs_sync(FIL *Log){
  FRESULT err = FR_OK;

  if (sync_tmo && fresh_data){
    err = f_sync(Log);
    sync_tmo = FALSE;
    fresh_data = FALSE;
  }
  return err;
}

/**
 * Get id of data
 * Pack it
 * Store to FS buffer
 * Raise bool flag if fresh data available
 */
FRESULT MavLogger::WriteLog(FIL *Log, mavMail *mail, bool *fresh_data) {
  uint32_t bytes_written;
  uint8_t *fs_buf;
  FRESULT err = FR_OK;
  size_t len;


  mavlink_message_t mavlink_msgbuf_log;
  uint8_t recordbuf[MAVLINK_SENDBUF_SIZE];



  mavlink_encode(mail->msgid, &mavlink_msgbuf_log, mail->mavmsg);
  mail->free();

  len = mavlink_msg_to_send_buffer(recordbuf, &mavlink_msgbuf_log);
  fs_buf = double_buf.append(recordbuf, len);
  if (fs_buf != nullptr) {
    err = f_write(Log, fs_buf, double_buf.size(), (UINT *)&bytes_written);
    if (err == FR_OK)
      *fresh_data = true;
  }



//#if MAVLINK_LOG_FORMAT
//#if CH_DBG_ENABLE_CHECKS
//  /* fill buffer with zeros except the timestamp region. Probably not necessary */
//  memset(recordbuf + TIME_LEN, 0, RECORD_LEN - TIME_LEN);
//#endif
//  uint64_t timestamp = pnsGetTimeUnixUsec();
//  mavlink_msg_to_send_buffer(recordbuf + TIME_LEN, &mavlink_msgbuf_log);
//  memcpy(recordbuf, &timestamp, TIME_LEN);
//  fs_buf = bufferize(recordbuf, RECORD_LEN);
//#else /* MAVLINK_LOG_FORMAT */
//  uint16_t len = 0;
//  len = mavlink_msg_to_send_buffer(recordbuf, &mavlink_msgbuf_log);
//  fs_buf = bufferize(recordbuf, len);
//#endif /* MAVLINK_LOG_FORMAT */
//
//  if (fs_buf != NULL){
//    err = f_write(Log, fs_buf, BUFF_SIZE, (UINT *)&bytes_written);
//    if (err == FR_OK)
//      *fresh_data = TRUE;
//  }
//
  return err;
}

/**
 * Если произошла ошибка - поток логгера просто тушится,
 * потому что исправить всё равно ничего нельзя.
 */
msg_t MavLogger::main(void){
  setName("MicroSD");

  FRESULT err;
  uint32_t clusters;
  FATFS *fsp;

  mavMail *mail;

  /* wait until card not ready */
NOT_READY:
  while (!sdcIsCardInserted(&SDCD1)) {
    if (this->shouldTerminate())
      goto EXIT;
    osalThreadSleepMilliseconds(SDC_POLLING_INTERVAL);
  }
  osalThreadSleepMilliseconds(SDC_POLLING_INTERVAL);
  if (!sdcIsCardInserted(&SDCD1))
    goto NOT_READY;
  else
    insert_handler();

  /* fs mounted? */
  if (!fs_ready)
    return MSG_RESET;

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

  while (this->shouldTerminate()) {
    /* wait ID */
    if (logwriter_mb.fetch(&mail, TIME_INFINITE) == MSG_OK){
      if (!sdcIsCardInserted(&SDCD1)){
        remove_handler();
        goto NOT_READY;
      }
      err = WriteLog(&log_file, mail, &fresh_data);
      err_check();
    }

    err = fs_sync(&log_file);
    err_check();
  }

EXIT:
  exit(MSG_OK);
  return MSG_OK;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 * @note    just drop message if logger not ready
 */
msg_t MavLogger::post(mavMail* msg) {
  msg_t ret = MSG_RESET;

  if (true == ready) {
    ret = logwriter_mb.post(msg, TIME_IMMEDIATE);
    if (MSG_OK != ret)
      this->drop_cnt++;
  }
  return ret;
}

/**
 *
 */
void MavLogger::stop(void) {
  this->requestTerminate();
  this->wait();
}
