#include <cstdlib>

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

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

#if 0
/*
Packet Type: 001 PMTK_ACK
Packet Meaning: Acknowledge Packet
DataFields:
  PktType: The packet type the acknowledge responds.
  Flag: ‘0’ = Invalid packet.
        ‘1’ = Unsupported packet type
        ‘2’ = Valid packet, but action failed
        ‘3’ = Valid packet, and action succeeded
Example:
  $PMTK001,101,0*33<CR><LF>
*/

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
#endif

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
static void set_fix_period(ProtoNmea &parser, char *buf, uint32_t period_ms) {
  char period_str[sizeof(period_ms) + 1];

  strcpy(buf, "$PMTK300,");
  strcat(buf, itoa(period_ms, period_str, 10));
  strcat(buf, ",0,0,0,0*");

  parser.seal(buf);
}

/**
 * 0 NMEA_SEN_GLL, // GPGLL interval - Geographic Position - Latitude longitude
 * 1 NMEA_SEN_RMC, // GPRMC interval - Recommended Minimum Specific GNSS Sentence
 * 2 NMEA_SEN_VTG, // GPVTG interval - Course Over Ground and Ground Speed
 * 3 NMEA_SEN_GGA, // GPGGA interval - GPS Fix Data
 * 4 NMEA_SEN_GSA, // GPGSA interval - GNSS DOPS and Active Satellites
 * 5 NMEA_SEN_GSV, // GPGSV interval - GNSS Satellites in View
 * 6 NMEA_SEN_GRS, // GPGRS interval - GNSS Range Residuals
 * 7 NMEA_SEN_GST, // GPGST interval - GNSS Pseudorange Errors Statistics
 *
 * supported frequency divider 0..5
 */
static void set_message_set(ProtoNmea &parser, char *buf, uint8_t set[17]) {

  char set_str[sizeof(set[0]) + 1];

  strcpy(buf, "$PMTK314");
  for (size_t i=0; i<17; i++) {
    osalDbgCheck(set[i] <= 5);
    strcat(buf, ",");
    strcat(buf, itoa(set[i], set_str, 10));
  }
  strcat(buf, "*");

  parser.seal(buf);
}

/**
 *
 */
static void set_baudrate(ProtoNmea &parser, char *buf, uint32_t baudrate) {

  char baudrate_str[sizeof(baudrate) + 1];

  strcpy(buf, "$PMTK251,");
  strcat(buf, itoa(baudrate, baudrate_str, 10));
  strcat(buf, "*");

  parser.seal(buf);
}

/**
 *
 */
void mtkgps::configure(void) {

  /* Tume message set. We have to do this some times
   * because serial port contains some garbage and this garbage will
   * be flushed out during sending of some messages */
  size_t i=3;
  while (i--) {
    uint8_t msgs[17];
    memset(msgs, 0, sizeof(msgs));
    msgs[1] = 1;
    msgs[3] = 1;
    msgs[5] = 5;

    memset(txbuf, 0, sizeof(txbuf));
    set_message_set(this->parser, txbuf, msgs);
    sdWrite(this->sdp, (uint8_t *)txbuf, strlen(txbuf));
    chThdSleepSeconds(1);
  }

  /* change baudrate if needed */
  if (this->start_baudrate != this->working_baudrate) {
    /* change _receiver_ baudrate */
    memset(txbuf, 0, sizeof(txbuf));
    set_baudrate(this->parser, txbuf, this->working_baudrate);
    sdWrite(this->sdp, (uint8_t *)txbuf, strlen(txbuf));

    /* change serial port baudrate */
    sdStop(this->sdp);
    gps_serial_cfg = {0, 0, 0, 0};
    gps_serial_cfg.speed = this->working_baudrate;
    sdStart(this->sdp, &gps_serial_cfg);

    chThdSleepSeconds(1);
  }

  /* set fix rate */
  memset(txbuf, 0, sizeof(txbuf));
  set_fix_period(this->parser, txbuf, *fix_period);
  sdWrite(this->sdp, (uint8_t *)txbuf, strlen(txbuf));
  fix_period_cache = *fix_period;
  chThdSleepSeconds(1);
}

/**
 *
 */
void mtkgps::update_settings(void) {

  if (fix_period_cache != *fix_period) {
    memset(txbuf, 0, sizeof(txbuf));
    set_fix_period(this->parser, txbuf, *fix_period);
    sdWrite(this->sdp, (uint8_t *)txbuf, sizeof(txbuf));
    fix_period_cache = *fix_period;
  }
}

/**
 *
 */
void mtkgps::load_params(void) {

  param_registry.valueSearch("GNSS_fix_period", &fix_period);
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
mtkgps::mtkgps(SerialDriver *sdp, uint32_t start_baudrate, uint32_t working_baudrate) :
    GenericNMEA(sdp, start_baudrate, working_baudrate) {
  return;
}

