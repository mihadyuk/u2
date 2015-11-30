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
void mtkgps::configure(void) {

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
void mtkgps::update_settings(void) {

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

