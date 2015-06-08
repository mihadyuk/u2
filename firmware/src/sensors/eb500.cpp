#include "main.h"

#include "nmea.hpp"
#include "mavlink_local.hpp"
#include "eb500.hpp"
#include "mav_logger.hpp"
#include "geometry.hpp"
#include "time_keeper.hpp"
#include "pads.h"
#include "chprintf.h"

using namespace gps;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define HDG_UNKNOWN             65535

#define GPS_DEFAULT_BAUDRATE    9600
#define GPS_HI_BAUDRATE         57600

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
chibios_rt::EvtSource event_gps;

extern mavlink_gps_raw_int_t        mavlink_out_gps_raw_int_struct;

extern MavLogger mav_logger;

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

__CCM__ static NmeaParser nmea_parser;

__CCM__ static nmea_gga_t gga;
__CCM__ static nmea_rmc_t rmc;
__CCM__ static gps_data_t cache;

static chibios_rt::BinarySemaphore pps_sem(true);
static chibios_rt::BinarySemaphore protect_sem(false);

static SerialDriver *hook_sdp = nullptr;

static mavMail gps_raw_int_mail;

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
static void log_append(void) {

  if (gps_raw_int_mail.free()) {
    gps_raw_int_mail.fill(&mavlink_out_gps_raw_int_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_GPS_RAW_INT);
    mav_logger.write(&gps_raw_int_mail);
  }
}


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

  log_append();
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
void gps_configure_mtk(void) {

  /* start on default baudrate */
  gps_ser_cfg.speed = GPS_DEFAULT_BAUDRATE;
  sdStart(&GPSSD, &gps_ser_cfg);

  /* set only GGA, RMC output. We have to do this some times
   * because serial port contains some garbage and this garbage will
   * be flushed out during sending of some messages */
  size_t i=3;
  while (i--) {
    sdWrite(&GPSSD, msg_gga_rmc_only, sizeof(msg_gga_rmc_only));
    chThdSleepSeconds(1);
  }

  /* set fix rate */
//  sdWrite(&GPSSD, fix_period_5hz, sizeof(fix_period_5hz));
//  sdWrite(&GPSSD, fix_period_4hz, sizeof(fix_period_4hz));
//  sdWrite(&GPSSD, fix_period_2hz, sizeof(fix_period_2hz));
  chThdSleepSeconds(1);

//  /* смена скорости _приемника_ на повышенную */
//  sdWrite(&GPSSD, gps_high_baudrate, sizeof(gps_high_baudrate));
//  chThdSleepSeconds(1);
//
//  /* перезапуск _порта_ на повышенной частоте */
//  sdStop(&GPSSD);
//  gps_ser_cfg.speed = GPS_HI_BAUDRATE;
//  sdStart(&GPSSD, &gps_ser_cfg);
//  chThdSleepSeconds(1);

  (void)fix_period_5hz;
  (void)fix_period_4hz;
  (void)fix_period_2hz;
  (void)gps_high_baudrate;
}

/**
 *
 */
void ubx_fletcher(const uint8_t *data, size_t len, uint8_t *result) {
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  for (size_t i=0; i<len; i++) {
    ck_a += data[i];
    ck_b += ck_a;
  }

  result[0] = ck_a;
  result[1] = ck_b;
}

/**
 * @brief   set solution period
 */
void ubx_solution_period(uint16_t msec) {
  uint8_t msg[84];
  memset(msg, 0, sizeof(msg));
  uint16_t tmp;
  osalDbgCheck(msec >= 200);

  msg[0] = 0xB5;
  msg[1] = 0x62;
  msg[2] = 0x06; // UBX-CFG
  msg[3] = 0x08; // RATE
  tmp = 6;
  memcpy(&msg[4],  &tmp, 2); //message length
  tmp = msec;
  memcpy(&msg[6],  &tmp, 2);
  tmp = 1;
  memcpy(&msg[8],  &tmp, 2);
  tmp = 0;
  memcpy(&msg[10], &tmp, 2);

  ubx_fletcher(&msg[2], 10, &msg[12]);

  sdWrite(&GPSSD, msg, 15);
}

/**
 *
 */
void ubx_message_decimate(const char *type, uint8_t rate) {
  char msg[84];
  memset(msg, 0, sizeof(msg));

  osalDbgCheck(3 == strlen(type));
  osalDbgCheck(rate <= 5);

  strcpy(msg, "$PUBX,40,---,0,0,0,0,0,0*");
  msg[9]  = type[0];
  msg[10] = type[1];
  msg[11] = type[2];
  msg[15] = rate + '0';

  nmea_parser.seal(msg);

  sdWrite(&GPSSD, (uint8_t*)msg, strlen(msg));
}

/**
 *
 */
void gps_configure_ubx(void) {

  /* start on default baudrate */
  gps_ser_cfg.speed = GPS_DEFAULT_BAUDRATE;
  sdStart(&GPSSD, &gps_ser_cfg);

  ubx_message_decimate("GLL", 0);
  ubx_message_decimate("GLL", 0);// hack: write message twice for port buffer cleaning
  ubx_message_decimate("GSV", 0);
  ubx_message_decimate("GSA", 0);
  ubx_message_decimate("VTG", 0);

  ubx_solution_period(200);
}

/**
 *
 */
static THD_WORKING_AREA(gpsRxThreadWA, 320) __CCM__;
THD_FUNCTION(gpsRxThread, arg) {
  chRegSetThreadName("GNSS");
  (void)arg;
  msg_t byte;
  sentence_type_t status;
  bool gga_acquired = false;
  bool rmc_acquired = false;
  systime_t prev = 0;
  systime_t curr = 0;

  osalThreadSleepSeconds(5);
  //gps_configure_mtk();
  gps_configure_ubx();

  while (!chThdShouldTerminateX()) {
    byte = sdGetTimeout(&GPSSD, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      status = nmea_parser.collect(byte);
      if (nullptr != hook_sdp)
        sdPut(hook_sdp, byte);

      switch(status) {
      case sentence_type_t::GGA:
        nmea_parser.unpack(gga);
        gga_acquired = true;
        break;
      case sentence_type_t::RMC:
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
        cache.altitude   = gga.altitude;
        cache.latitude   = gga.latitude;
        cache.longitude  = gga.longitude;
        cache.course     = rmc.course;
        cache.speed      = rmc.speed;
        cache.time       = rmc.time;
        cache.sec_round  = rmc.sec_round;
        if (gga.fix == 1) {
          event_gps.broadcastFlags(EVMSK_GPS_FRESH_VALID);
          if (nullptr != hook_sdp) {
            curr = chVTGetSystemTimeX();
            systime_t out = ST2MS(curr - prev);
            prev = curr;
            chprintf((BaseSequentialStream *)hook_sdp, "%U\r\n", out);
          }
        }
        release();
      }
    }
  }

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
void GPSInit(void){

  chThdCreateStatic(gpsRxThreadWA, sizeof(gpsRxThreadWA),
                    GPSPRIO, gpsRxThread, NULL);
}

/**
 *
 */
void GPSSetDumpHook(SerialDriver *sdp) {

  hook_sdp = sdp;
}

/**
 *
 */
void GPSDeleteDumpHook(void) {

  hook_sdp = nullptr;
}

/**
 *
 */
void GPSGet(gps_data_t &result) {

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
