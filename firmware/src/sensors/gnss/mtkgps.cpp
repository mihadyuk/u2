#include "main.h"

#include "mavlink_local.hpp"
#include "mtkgps.hpp"
#include "mav_logger.hpp"
#include "geometry.hpp"
#include "time_keeper.hpp"
#include "pads.h"
#include "chprintf.h"
#include "array_len.hpp"
#include "param_registry.hpp"

using namespace gnss;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_gps_raw_int_t        mavlink_out_gps_raw_int_struct;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
/**
 *
 */
static SerialConfig gps_ser_cfg = {
    GNSS_DEFAULT_BAUDRATE,
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

/* constants for NMEA parser needs */
static const uint16_t GGA_VOID = 0xFFFF;
static const uint16_t RMC_VOID = (0xFFFF - 1);

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
void mtkgps::ggarmc2mavlink(const nmea_gga_t &gga, const nmea_rmc_t &rmc) {

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

  log_append(&mavlink_out_gps_raw_int_struct);
}

/**
 *
 */
void mtkgps::configure(void) {

  /* start on default baudrate */
  gps_ser_cfg.speed = GNSS_DEFAULT_BAUDRATE;
  sdStart(this->sdp, &gps_ser_cfg);

  /* set only GGA, RMC output. We have to do this some times
   * because serial port contains some garbage and this garbage will
   * be flushed out during sending of some messages */
  size_t i=3;
  while (i--) {
    sdWrite(this->sdp, msg_gga_rmc_only, sizeof(msg_gga_rmc_only));
    chThdSleepSeconds(1);
  }

  /* set fix rate */
  sdWrite(this->sdp, fix_period_5hz, sizeof(fix_period_5hz));
//  sdWrite(this->sdp, fix_period_4hz, sizeof(fix_period_4hz));
//  sdWrite(this->sdp, fix_period_2hz, sizeof(fix_period_2hz));
  chThdSleepSeconds(1);

//  /* смена скорости _приемника_ на повышенную */
//  sdWrite(this->sdp, gps_high_baudrate, sizeof(gps_high_baudrate));
//  chThdSleepSeconds(1);
//
//  /* перезапуск _порта_ на повышенной частоте */
//  sdStop(this->sdp);
//  gps_ser_cfg.speed = GPS_HI_BAUDRATE;
//  sdStart(this->sdp, &gps_ser_cfg);
//  chThdSleepSeconds(1);

  (void)fix_period_5hz;
  (void)fix_period_4hz;
  (void)fix_period_2hz;
  (void)gps_high_baudrate;
}

/**
 *
 */
static void gnss_unpack(const nmea_gga_t &gga, const nmea_rmc_t &rmc,
                        gnss_data_t *result) {

  if (false == result->fresh) {
    result->altitude   = gga.altitude;
    result->latitude   = gga.latitude;
    result->longitude  = gga.longitude;
    result->course     = rmc.course;
    result->speed      = rmc.speed;
    result->speed_type = GNSS_SPEED_TYPE_SPEED_COURSE;
    result->time       = rmc.time;
    result->msec       = rmc.msec;
    result->fix        = gga.fix;
    result->fresh      = true; // this line must be at the very end for atomicity
  }
}

/**
 *
 */
void mtkgps::update_settings(void) {
  return;
}

/**
 *
 */
THD_FUNCTION(mtkgps::nmeaRxThread, arg) {
  chRegSetThreadName("GNSS_NMEA");
  mtkgps *self = static_cast<mtkgps *>(arg);
  msg_t byte;
  sentence_type_t status;
  nmea_gga_t gga;
  nmea_rmc_t rmc;
  uint16_t gga_msec = GGA_VOID;
  uint16_t rmc_msec = RMC_VOID;

  osalThreadSleepSeconds(5);
  self->configure();

  while (!chThdShouldTerminateX()) {
    self->update_settings();

    byte = sdGetTimeout(self->sdp, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      status = self->nmea_parser.collect(byte);
      if (nullptr != self->sniff_sdp)
        sdPut(self->sniff_sdp, byte);

      switch(status) {
      case sentence_type_t::GGA:
        self->nmea_parser.unpack(gga);
        gga_msec = gga.msec + gga.time.tm_sec * 1000;
        if (nullptr != self->sniff_sdp) {
          chprintf((BaseSequentialStream *)self->sniff_sdp, "gga_parsed = %u\n", gga.msec);
        }
        break;
      case sentence_type_t::RMC:
        self->nmea_parser.unpack(rmc);
        rmc_msec = rmc.msec + rmc.time.tm_sec * 1000;
        if (nullptr != self->sniff_sdp) {
          chprintf((BaseSequentialStream *)self->sniff_sdp, "rmc_parsed = %u\n", rmc.msec);
        }
        break;
      default:
        break;
      }

      /* */
      //if ((gga_msec != rmc_msec) && (gga_msec != GGA_VOID) && (rmc_msec != RMC_VOID)) { // test string
      if (gga_msec == rmc_msec) { // correct string

        self->ggarmc2mavlink(gga, rmc);

        self->acquire();
        for (size_t i=0; i<ArrayLen(self->spamlist); i++) {
          if (nullptr != self->spamlist[i]) {
            gnss_unpack(gga, rmc, self->spamlist[i]);
          }
        }
        self->release();

        if (gga.fix == 1) {
          event_gnss.broadcastFlags(EVMSK_GNSS_FRESH_VALID);
        }

        if (nullptr != self->sniff_sdp) {
          chprintf((BaseSequentialStream *)self->sniff_sdp, "---- gga = %u; rmc = %u\n", gga_msec, rmc_msec);
        }

        gga_msec = GGA_VOID;
        rmc_msec = RMC_VOID;
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
mtkgps::mtkgps(SerialDriver *sdp) : GNSSReceiver(sdp) {
  return;
}

/**
 *
 */
void mtkgps::start(void) {

  param_registry.valueSearch("GNSS_fix_period", &fix_period);

  worker = chThdCreateStatic(gnssRxThreadWA, sizeof(gnssRxThreadWA),
                      GPSPRIO, nmeaRxThread, this);
  osalDbgCheck(nullptr != worker);
  ready = true;
}
