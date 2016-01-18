#pragma GCC optimize "-O2"

#include "main.h"

#include "generic_nmea.hpp"
#include "mavlink_local.hpp"
#include "mav_logger.hpp"
#include "geometry.hpp"
#include "time_keeper.hpp"
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
extern mavlink_gps_status_t         mavlink_out_gps_status_struct;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/* constants for NMEA parser needs */
static const uint16_t GGA_VOID = 0xFFFF;
static const uint16_t RMC_VOID = (0xFFFF - 1);
__CCM__ static nmea_gsv_t gl_gsv[5];
__CCM__ static nmea_gsv_t gp_gsv[5];
__CCM__ static bool gl_gsv_fresh = false;
__CCM__ static bool gp_gsv_fresh = false;

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
static void gsvfield2mavlink(const nmea_gsv_satellite_t *sat, size_t N) {

  if (nullptr == sat) {
    mavlink_out_gps_status_struct.satellite_azimuth[N]    = 0;
    mavlink_out_gps_status_struct.satellite_elevation[N]  = 0;
    mavlink_out_gps_status_struct.satellite_prn[N]        = 0;
    mavlink_out_gps_status_struct.satellite_snr[N]        = 0;
    mavlink_out_gps_status_struct.satellite_used[N]       = 0;
    return;
  }

  if (N < ArrayLen(mavlink_out_gps_status_struct.satellite_azimuth)) { // prevent overflow
    float a = sat->azimuth * 0.708333f;
    mavlink_out_gps_status_struct.satellite_azimuth[N]    = roundf(a);
    mavlink_out_gps_status_struct.satellite_elevation[N]  = sat->elevation;
    mavlink_out_gps_status_struct.satellite_prn[N]        = sat->id;
    mavlink_out_gps_status_struct.satellite_snr[N]        = sat->snr;
    mavlink_out_gps_status_struct.satellite_used[N]       = 0; // unknown
  }
}

/**
 *
 */
static void gsv2mavlink(const nmea_gsv_t *gp, size_t P, const nmea_gsv_t *gl, size_t L) {
  size_t total = 0;

  mavlink_out_gps_status_struct.satellites_visible = gp[0].sat_visible + gl[0].sat_visible;

  /* first process GPS*/
  for (size_t i=0; i<P; i++) {
    for (size_t j=0; j<ArrayLen(gp[0].sat); j++) {
      const nmea_gsv_satellite_t *sat = &gp[i].sat[j];
      if (sat->id != 0) {
        gsvfield2mavlink(sat, total);
        total++;
      }
    }
  }

  /* Glonass */
  for (size_t i=0; i<L; i++) {
    for (size_t j=0; j<ArrayLen(gl[0].sat); j++) {
      const nmea_gsv_satellite_t *sat = &gl[i].sat[j];
      if (sat->id != 0) {
        gsvfield2mavlink(sat, total);
        total++;
      }
    }
  }

  /* clean unused fields */
  while (total < ArrayLen(mavlink_out_gps_status_struct.satellite_azimuth)) {
    gsvfield2mavlink(nullptr, total);
    total++;
  }
}

/**
 *
 */
void GenericNMEA::ggarmc2mavlink(const nmea_gga_t &gga, const nmea_rmc_t &rmc) {

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
void GenericNMEA::gnss_unpack(const nmea_gga_t &gga, const nmea_rmc_t &rmc,
                               gnss_data_t *result) {

  result->altitude   = gga.altitude;
  result->latitude   = deg2rad(gga.latitude);
  result->longitude  = deg2rad(gga.longitude);
  result->course     = deg2rad(rmc.course);
  result->speed      = rmc.speed;
  result->speed_type = speed_t::SPEED_COURSE;
  result->time       = rmc.time;
  result->msec       = rmc.msec;
  result->fix        = gga.fix;
}

/**
 *
 */
void GenericNMEA::dispatch_data(const nmea_gga_t &gga, const nmea_rmc_t &rmc) {

  this->acquire();

  for (size_t i=0; i<ArrayLen(this->spamlist); i++) {
    gnss_data_t* p = this->spamlist[i];
    if ((nullptr != p) && (false == p->fresh)) {
      this->gnss_unpack(gga, rmc, p);
      p->fresh = true; // must be at the very end for atomicity
    }
  }

  this->release();
}

/**
 *
 */
void GenericNMEA::configure(void) {
  osalDbgAssert(this->start_baudrate == this->working_baudrate,
      "Generic NMEA receiver does not allow different baudrates");
}

/**
 *
 */
THD_FUNCTION(GenericNMEA::nmeaRxThread, arg) {
  chRegSetThreadName("GNSS_NMEA");
  GenericNMEA *self = static_cast<GenericNMEA *>(arg);
  msg_t byte;
  sentence_type_t status;
  uint16_t gga_msec = GGA_VOID;
  uint16_t rmc_msec = RMC_VOID;
  nmea_gga_t gga;
  nmea_rmc_t rmc;
  nmea_gsv_t gsv;

  osalThreadSleepSeconds(4);
  self->configure();
  self->subscribe_inject();

  while (!chThdShouldTerminateX()) {
    self->update_settings();

    byte = sdGetTimeout(self->sdp, MS2ST(50));
    if (MSG_TIMEOUT != byte) {
      status = self->parser.collect(byte);
      if (nullptr != self->sniff_chnp)
        chnPutTimeout(self->sniff_chnp, byte, TIME_IMMEDIATE);

      /* main receiving switch */
      switch(status) {
      case sentence_type_t::GGA:
        self->parser.unpack(gga);
        gga_msec = gga.msec + gga.time.tm_sec * 1000;
        break;
      case sentence_type_t::RMC:
        self->parser.unpack(rmc);
        rmc_msec = rmc.msec + rmc.time.tm_sec * 1000;
        break;
      case sentence_type_t::GPGSV:
        self->parser.unpack(gsv);
        if (1 == gsv.current)
          memset(gp_gsv, 0, sizeof(gp_gsv));
        gp_gsv[gsv.current - 1] = gsv;
        if (gsv.current == gsv.total)
          gp_gsv_fresh = true;
        break;
      case sentence_type_t::GLGSV:
        self->parser.unpack(gsv);
        if (1 == gsv.current)
          memset(gl_gsv, 0, sizeof(gl_gsv));
        gl_gsv[gsv.current - 1] = gsv;
        if (gsv.current == gsv.total)
          gl_gsv_fresh = true;
        break;
      default:
        break;
      }

      /* GSV data */
      if (gga.satellites > 0) {
        if (gl_gsv_fresh & gp_gsv_fresh) {
          gsv2mavlink(gp_gsv, ArrayLen(gp_gsv), gl_gsv, ArrayLen(gl_gsv));
          gp_gsv_fresh = false;
          gl_gsv_fresh = false;
        }
      }
      else {
        /* work around missing GSV messages */
        memset(&mavlink_out_gps_status_struct, 0, sizeof(mavlink_out_gps_status_struct));
      }

      /* Workaround. Prevents mixing of speed and coordinates from different
         measurements. */
      if ((gga_msec == rmc_msec) && (GGA_VOID != gga_msec) && (RMC_VOID != rmc_msec)) {
        self->ggarmc2mavlink(gga, rmc);
        self->dispatch_data(gga, rmc);
        gga_msec = GGA_VOID;
        rmc_msec = RMC_VOID;

        if (gga.fix > 0)
          event_gnss.broadcastFlags(EVMSK_GNSS_FRESH_VALID);
      }
    }

    /* GNSS assistance. Must be implemented in derivative classes */
    self->inject();
  }

  self->release_inject();
  chThdExit(MSG_OK);
}

/**
 *
 */
void GenericNMEA::start_impl(void) {

  load_params();

  worker = chThdCreateStatic(gnssRxThreadWA, sizeof(gnssRxThreadWA),
                      GPSPRIO, nmeaRxThread, this);
  osalDbgCheck(nullptr != worker);
  ready = true;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
GenericNMEA::GenericNMEA(SerialDriver *sdp, uint32_t start_baudrate, uint32_t working_baudrate) :
    GNSSReceiver(sdp, start_baudrate, working_baudrate) {
  return;
}


