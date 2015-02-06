#include "main.h"

#include "nmea.hpp"
#include "global_flags.h"
#include "mavlink_local.hpp"
#include "gps_eb500.hpp"
#include "bkp.hpp"
#include "geometry.hpp"
#include "exti_local.hpp"

using namespace gps;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define HDG_UNKNOWN     65535

#define GPS_DEFAULT_BAUDRATE 9600
#define GPS_HI_BAUDRATE 57600

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
static SerialConfig gps_ser_cfg = {
    GPS_DEFAULT_BAUDRATE,
    0,
    0,
    0,
};

static const uint8_t msg_gga_rmc_only[] =
    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
// подтверждение
//$PMTK001,314,3*36

// время между фиксацией координат в милисекундах
static const uint8_t fix_period_5hz[] =
    "$PMTK300,200,0,0,0,0*2F\r\n";
// подтверждение
// $PMTK001,300,3*33

// установка скорости обмена
static const uint8_t gps_high_baudrate[] =
    "$PMTK251,57600*2C\r\n";

static nmeap_gga_t gga __attribute__((section(".ccm")));
static nmeap_rmc_t rmc __attribute__((section(".ccm")));
static NmeaParser nmea_parser __attribute__((section(".ccm")));

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/* рассчет количества милисекунд, необходимых для отправки заданного количества
 байт на заданном бодрейте. 13 - это количество бод в одном байте (взято
 с запасом). 2 - это дополнительные запасные милисекунды */
static systime_t byte2msec(size_t bytes, size_t baudrate) {
  return ((bytes * 13 * 1000) / baudrate) + 2;
}

/**
 *
 */
static void gps_configure(void) {
  /* запуск на дефолтной частоте */
  gps_ser_cfg.speed = GPS_DEFAULT_BAUDRATE;
  sdStart(&GPSSD, &gps_ser_cfg);

  /* смена скорости ПРИЁМНИКА на повышенную */
  sdWrite(&GPSSD, gps_high_baudrate, sizeof(gps_high_baudrate));
  chThdSleepMilliseconds(byte2msec(sizeof(gps_high_baudrate), GPS_DEFAULT_BAUDRATE));

  /* перезапуск порта контроллера на повышенной частоте */
  sdStop(&GPSSD);
  gps_ser_cfg.speed = GPS_HI_BAUDRATE;
  sdStart(&GPSSD, &gps_ser_cfg);

  /* установка выдачи только GGA и RMC */
  sdWrite(&GPSSD, msg_gga_rmc_only, sizeof(msg_gga_rmc_only));
  chThdSleepMilliseconds(byte2msec(sizeof(msg_gga_rmc_only), GPS_DEFAULT_BAUDRATE));

  /* установка частоты обновления 5 Гц */
  sdWrite(&GPSSD, fix_period_5hz, sizeof(fix_period_5hz));
  chThdSleepMilliseconds(byte2msec(sizeof(fix_period_5hz), GPS_DEFAULT_BAUDRATE));

  /* зачистка входной очереди после всех манипуляций. На всякий случай */
  //  chSysLock();
  //  chIQResetI( &(GPSSD.iqueue));
  //  chSysUnlock();
}

/**
 *
 */
static THD_WORKING_AREA(gpsRxThreadWA, 128) __attribute__((section(".ccm")));
static msg_t gpsRxThread(void *arg) {
  chRegSetThreadName("gpsRx");
  (void)arg;
  msg_t byte;
  collect_status_t status;

  osalThreadSleepSeconds(5);
  gps_configure();

  while (!chThdShouldTerminateX()) {
    byte = sdGetTimeout(&GPSSD, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      status = nmea_parser.collect(byte);

      switch(status) {
      case collect_status_t::GPGGA:
        nmea_parser.unpack(&gga);
        break;

      case collect_status_t::GPRMC:
        nmea_parser.unpack(&rmc);
        break;

      default:
        break;
      }
    }
  }

  chThdExit(MSG_OK);
  return MSG_OK;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
void GPSInit(void){

  chThdCreateStatic(gpsRxThreadWA, sizeof(gpsRxThreadWA), GPSPRIO,
                    gpsRxThread, NULL);
}

