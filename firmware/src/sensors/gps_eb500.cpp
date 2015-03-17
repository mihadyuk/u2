#include "main.h"

#include "nmea.hpp"
#include "mavlink_local.hpp"
#include "gps_eb500.hpp"
#include "geometry.hpp"
#include "time_keeper.hpp"

using namespace gps;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define HDG_UNKNOWN             65535

#define GPS_DEFAULT_BAUDRATE    9600
#define GPS_HI_BAUDRATE         57600

#define DEG_TO_MAVLINK          (10 * 1000 * 1000)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
chibios_rt::EvtSource event_gps;

extern mavlink_gps_raw_int_t           mavlink_out_gps_raw_int_struct;
extern mavlink_global_position_int_t   mavlink_out_global_position_int_struct;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
/**
 *
 */
static SerialConfig gps_ser_cfg = {
    GPS_DEFAULT_BAUDRATE,
    0,
    0,
    0,
};

static const uint8_t msg_gga_rmc_only[] =
    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
// confirmation: $PMTK001,314,3*36

/* time between fixes (mS) */
static const uint8_t fix_period_5hz[] = "$PMTK300,200,0,0,0,0*2F\r\n";
static const uint8_t fix_period_4hz[] = "$PMTK300,250,0,0,0,0*2A\r\n";
static const uint8_t fix_period_2hz[] = "$PMTK300,500,0,0,0,0*28\r\n";
// confirmation: $PMTK001,300,3*33

/* set serial port baudrate */
static const uint8_t gps_high_baudrate[] = "$PMTK251,57600*2C\r\n";

static NmeaParser nmea_parser __attribute__((section(".ccm")));

static nmea_gga_t gga   __attribute__((section(".ccm")));
static nmea_rmc_t rmc   __attribute__((section(".ccm")));
static gps_data_t cache __attribute__((section(".ccm")));

static chibios_rt::BinarySemaphore pps_sem(true);
static chibios_rt::BinarySemaphore protect_sem(false);

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

/**
 *
 */
static void gps2mavlink(const nmea_gga_t &gga, const nmea_rmc_t &rmc) {

  mavlink_out_gps_raw_int_struct.time_usec = TimeKeeper::utc();
  mavlink_out_gps_raw_int_struct.lat = gga.latitude  * DEG_TO_MAVLINK;
  mavlink_out_gps_raw_int_struct.lon = gga.longitude * DEG_TO_MAVLINK;
  mavlink_out_gps_raw_int_struct.alt = gga.altitude * 1000;
  mavlink_out_gps_raw_int_struct.eph = gga.hdop * 100;
  mavlink_out_gps_raw_int_struct.epv = UINT16_MAX;
  mavlink_out_gps_raw_int_struct.fix_type = gga.fix;
  mavlink_out_gps_raw_int_struct.satellites_visible = gga.satellites;
  mavlink_out_gps_raw_int_struct.cog = rmc.course * 100;
  mavlink_out_gps_raw_int_struct.vel = rmc.speed * 100;

  mavlink_out_global_position_int_struct.time_boot_ms = TIME_BOOT_MS;
  mavlink_out_global_position_int_struct.alt = mavlink_out_gps_raw_int_struct.alt;
  mavlink_out_global_position_int_struct.lat = mavlink_out_gps_raw_int_struct.lat;
  mavlink_out_global_position_int_struct.lon = mavlink_out_gps_raw_int_struct.lon;
  mavlink_out_global_position_int_struct.hdg = mavlink_out_gps_raw_int_struct.cog;
}

/**
 *
 */
static void acquire(void) {
  protect_sem.wait();
}

/**
 *
 */
static void release(void) {
  protect_sem.signal();
}

/**
 *
 */
static void gps_configure(void) {
  /* запуск на дефолтной частоте */
  gps_ser_cfg.speed = GPS_DEFAULT_BAUDRATE;
  sdStart(&GPSSD, &gps_ser_cfg);

//  /* смена скорости ПРИЁМНИКА на повышенную */
//  sdWrite(&GPSSD, gps_high_baudrate, sizeof(gps_high_baudrate));
//  chThdSleepSeconds(1);
//
//  /* перезапуск порта контроллера на повышенной частоте */
//  sdStop(&GPSSD);
//  gps_ser_cfg.speed = GPS_HI_BAUDRATE;
//  sdStart(&GPSSD, &gps_ser_cfg);

  /* установка выдачи только GGA и RMC */
  sdWrite(&GPSSD, msg_gga_rmc_only, sizeof(msg_gga_rmc_only));
  chThdSleepSeconds(1);

  /* установка частоты обновления */
  sdWrite(&GPSSD, fix_period_5hz, sizeof(fix_period_5hz));
//  sdWrite(&GPSSD, fix_period_4hz, sizeof(fix_period_4hz));
//  sdWrite(&GPSSD, fix_period_2hz, sizeof(fix_period_2hz));
  chThdSleepSeconds(1);

  (void)fix_period_5hz;
  (void)fix_period_4hz;
  (void)fix_period_2hz;
  (void)gps_high_baudrate;
}

/**
 *
 */
static THD_WORKING_AREA(gpsRxThreadWA, 320) __attribute__((section(".ccm")));
static msg_t gpsRxThread(void *arg) {
  chRegSetThreadName("gpsRx");
  (void)arg;
  msg_t byte;
  collect_status_t status;
  bool gga_acquired = false;
  bool rmc_acquired = false;

  osalThreadSleepSeconds(5);
  gps_configure();

  while (!chThdShouldTerminateX()) {
    byte = sdGetTimeout(&GPSSD, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      status = nmea_parser.collect(byte);

      switch(status) {
      case collect_status_t::GPGGA:
        nmea_parser.unpack(gga);
        gga_acquired = true;
        break;
      case collect_status_t::GPRMC:
        nmea_parser.unpack(rmc);
        rmc_acquired = true;
        break;
      default:
        break;
      }

      /* */
      if (gga_acquired && rmc_acquired) {
        gga_acquired = false;
        rmc_acquired = false;

        gps2mavlink(gga, rmc);

        acquire();
        if (gga.fix > 0)
          cache.fix_valid = true;
        else
          cache.fix_valid = false;
        cache.altitude   = gga.altitude;
        cache.latitude   = gga.latitude;
        cache.longitude  = gga.longitude;
        cache.course     = rmc.course;
        cache.speed      = rmc.speed;
        cache.time       = rmc.time;
        cache.sec_round  = rmc.sec_round;
        release();

        event_gps.broadcastFlags(EVMSK_GPS_UPATED);
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

  chThdCreateStatic(gpsRxThreadWA, sizeof(gpsRxThreadWA),
                    GPSPRIO, gpsRxThread, NULL);
}

/**
 *
 */
void GPSGetData(gps_data_t &result) {

  acquire();
  result = cache;
  release();
}

/**
 *
 */
void GPS_PPS_ISR_I(void) {

  pps_sem.signalI();
}
