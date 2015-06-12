#include "main.h"

#include "nmea_proto.hpp"
#include "ubx_proto.hpp"
#include "mavlink_local.hpp"
#include "gnss_receiver.hpp"
#include "mav_logger.hpp"
#include "geometry.hpp"
#include "time_keeper.hpp"
#include "pads.h"
#include "chprintf.h"
#include "array_len.hpp"

using namespace gnss;

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

__CCM__ static NmeaProto nmea_parser;
__CCM__ static UbxProto  ubx_parser;

static chibios_rt::BinarySemaphore pps_sem(true);
static chibios_rt::BinarySemaphore protect_sem(false);

static mavMail gps_raw_int_mail;

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
 * @brief   set solution period
 */
static void ubx_solution_period(uint16_t msec) {
  uint8_t buf[32];
  ubx_cfg_rate msg;
  size_t len;

  osalDbgCheck(msec >= 200);

  msg.measRate = msec;
  msg.navRate = 1;
  msg.timeRef = 0;

  len = ubx_parser.pack(msg, buf, sizeof(buf));
  osalDbgCheck(len > 0 && len <= sizeof(buf));

  sdWrite(&GPSSD, buf, len);
}

/**
 *
 */
static void ubx_message_decimate(const char *type, uint8_t rate) {
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
static void gnss_unpack(const nmea_gga_t &gga, const nmea_rmc_t &rmc,
                        gnss_data_t *result) {

  if (false == result->fresh) {
    result->altitude   = gga.altitude;
    result->latitude   = gga.latitude;
    result->longitude  = gga.longitude;
    result->course     = rmc.course;
    result->speed      = rmc.speed;
    result->speed_type = speed_t::SPEED_COURSE;
    result->time       = rmc.time;
    result->sec_round  = rmc.msec == 0;
    result->fix        = gga.fix;
    result->fresh      = true; // this line must be at the very end
  }
}

/**
 *
 */
static THD_WORKING_AREA(gnssRxThreadWA, 400) __CCM__;
THD_FUNCTION(GNSSReceiver::nmeaRxThread, arg) {
  chRegSetThreadName("GNSS_NMEA");
  GNSSReceiver *self = static_cast<GNSSReceiver *>(arg);
  msg_t byte;
  sentence_type_t status;
  nmea_gga_t gga;
  nmea_rmc_t rmc;
  uint16_t gga_msec = GGA_VOID;
  uint16_t rmc_msec = RMC_VOID;

  osalThreadSleepSeconds(5);
  //gps_configure_mtk();
  gps_configure_ubx();

  while (!chThdShouldTerminateX()) {
    byte = sdGetTimeout(&GPSSD, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      status = nmea_parser.collect(byte);
      if (nullptr != self->sniff_sdp)
        sdPut(self->sniff_sdp, byte);

      switch(status) {
      case sentence_type_t::GGA:
        nmea_parser.unpack(gga);
        gga_msec = gga.msec + gga.time.tm_sec * 1000;
        if (nullptr != self->sniff_sdp) {
          chprintf((BaseSequentialStream *)self->sniff_sdp, "gga_parsed = %u\n", gga.msec);
        }
        break;
      case sentence_type_t::RMC:
        nmea_parser.unpack(rmc);
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

        gps2mavlink(gga, rmc);

        acquire();
        for (size_t i=0; i<ArrayLen(self->spamlist); i++) {
          if (nullptr != self->spamlist[i]) {
            gnss_unpack(gga, rmc, self->spamlist[i]);
          }
        }
        release();

        if (gga.fix == 1) {
          event_gps.broadcastFlags(EVMSK_GNSS_FRESH_VALID);
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

/**
 *
 */
THD_FUNCTION(GNSSReceiver::ubxRxThread, arg) {
  chRegSetThreadName("GNSS_UBX");
  GNSSReceiver *self = static_cast<GNSSReceiver *>(arg);
  (void)self;
  msg_t byte;
  ubx_msg_t status;
  ubx_nav_posllh posllh;
  ubx_nav_velned velned;

  osalThreadSleepSeconds(3);
  gps_configure_ubx();

  while (!chThdShouldTerminateX()) {
    byte = sdGetTimeout(&GPSSD, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      status = ubx_parser.collect(byte);

      switch(status) {
      case ubx_msg_t::EMPTY:
        break;
      case ubx_msg_t::NAV_POSLLH:
        ubx_parser.unpack(posllh);
        break;
      case ubx_msg_t::NAV_VELNED:
        ubx_parser.unpack(velned);
        break;
      default:
        ubx_parser.drop(); // it is essential to drop unneded message
        break;
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
GNSSReceiver::GNSSReceiver(void) {
  return;
}

/**
 *
 */
void GNSSReceiver::start(void) {

  worker = chThdCreateStatic(gnssRxThreadWA, sizeof(gnssRxThreadWA),
                    GPSPRIO, nmeaRxThread, this);
  osalDbgCheck(nullptr != worker);
  ready = true;
}

/**
 *
 */
void GNSSReceiver::stop(void) {
  ready = false;

  chThdTerminate(worker);
  chThdWait(worker);
  worker = nullptr;
}

/**
 *
 */
void GNSSReceiver::setSniffer(SerialDriver *sdp) {
  osalDbgCheck(nullptr != sdp);
  this->sniff_sdp = sdp;
}

/**
 *
 */
void GNSSReceiver::deleteSniffer(void) {

  this->sniff_sdp = nullptr;
}

/**
 *
 */
void GNSSReceiver::subscribe(gnss_data_t* result) {
  osalDbgCheck(nullptr != result);

  acquire();

  for (size_t i=0; i<ArrayLen(spamlist); i++) {
    osalDbgAssert(result != spamlist[i],
        "you can not subscribe single structure twice");
  }

  for (size_t i=0; i<ArrayLen(spamlist); i++) {
    if (nullptr == spamlist[i]) {
      spamlist[i] = result;
      release();
      return;
    }
  }

  release();
  osalSysHalt("No free slots remain");
}

/**
 *
 */
void GNSSReceiver::unsubscribe(gnss_data_t* result) {
  osalDbgCheck(nullptr != result);

  acquire();

  for (size_t i=0; i<ArrayLen(spamlist); i++) {
    if (result == spamlist[i]) {
      spamlist[i] = nullptr;
      release();
      return;
    }
  }

  release();
  osalSysHalt("This message not subscribed");
}



/**
 *
 */
void GNSSReceiver::getCache(gnss_data_t &result) {

  osalSysLock();
  result = this->cache;
  osalSysUnlock();
}

/**
 *
 */
void GNSSReceiver::GNSS_PPS_ISR_I(void) {

  pps_sem.signalI();
}
