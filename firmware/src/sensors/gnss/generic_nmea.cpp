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

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

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

  if (false == result->fresh) {
    result->altitude   = gga.altitude;
    result->latitude   = gga.latitude;
    result->longitude  = gga.longitude;
    result->course     = rmc.course;
    result->speed      = rmc.speed;
    result->speed_type = speed_t::SPEED_COURSE;
    result->time       = rmc.time;
    result->msec       = rmc.msec;
    result->fix        = gga.fix;
    result->fresh      = true; // this line must be at the very end for atomicity
  }
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
  nmea_gga_t gga;
  nmea_rmc_t rmc;
  uint16_t gga_msec = GGA_VOID;
  uint16_t rmc_msec = RMC_VOID;

  osalThreadSleepSeconds(5);
  self->configure();
  self->subscribe_assistance();

  while (!chThdShouldTerminateX()) {
    self->update_settings();

    byte = sdGetTimeout(self->sdp, MS2ST(50));
    if (MSG_TIMEOUT != byte) {
      status = self->nmea_parser.collect(byte);
      if (nullptr != self->sniff_sdp)
        sdPut(self->sniff_sdp, byte);

      switch(status) {
      case sentence_type_t::GGA:
        self->nmea_parser.unpack(gga);
        gga_msec = gga.msec + gga.time.tm_sec * 1000;
        if (nullptr != self->sniff_sdp) {
          chprintf((BaseSequentialStream *)self->sniff_sdp,
              "---- gga_parsed = %u\n", gga.msec);
        }
        break;
      case sentence_type_t::RMC:
        self->nmea_parser.unpack(rmc);
        rmc_msec = rmc.msec + rmc.time.tm_sec * 1000;
        if (nullptr != self->sniff_sdp) {
          chprintf((BaseSequentialStream *)self->sniff_sdp,
              "---- rmc_parsed = %u\n", rmc.msec);
        }
        break;
      default:
        break;
      }

      /* Workaround. Prevents mixing of speed and coordinates from different
       * measurements. */
      if (gga_msec == rmc_msec) {

        self->ggarmc2mavlink(gga, rmc);

        self->acquire();
        for (size_t i=0; i<ArrayLen(self->spamlist); i++) {
          if (nullptr != self->spamlist[i]) {
            self->gnss_unpack(gga, rmc, self->spamlist[i]);
          }
        }
        self->release();

        if (gga.fix == 1) {
          event_gnss.broadcastFlags(EVMSK_GNSS_FRESH_VALID);
        }

        if (nullptr != self->sniff_sdp) {
          chprintf((BaseSequentialStream *)self->sniff_sdp,
              "---- gga = %u; rmc = %u\n", gga_msec, rmc_msec);
        }

        gga_msec = GGA_VOID;
        rmc_msec = RMC_VOID;
      }
    }

    /* GNSS assistance. Must be implemented in derivative classes */
    self->assist();
  }

  self->release_assistance();
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

/**
 * @brief   Takes assistant messages (RTCM or whatever receiver accepts)
 *          and push it to serial port
 */
//bool GenericNMEA::assist(const uint8_t *data, size_t len) {
//
//}



