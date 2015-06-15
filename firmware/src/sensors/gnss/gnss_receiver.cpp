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
#include "param_registry.hpp"

using namespace gnss;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define HDG_UNKNOWN             65535

#define GNSS_DEFAULT_BAUDRATE    9600
#define GNSS_HI_BAUDRATE         57600

/**
 *
 */
enum class ubx_ack_t {
  NACK  = 0,
  ACK   = 1,
  NONE  = 2
};

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
static void gps2mavlink(const ubx_nav_pvt &pvt) {

  mavlink_out_gps_raw_int_struct.time_usec = TimeKeeper::utc();
  mavlink_out_gps_raw_int_struct.lat = pvt.lat;
  mavlink_out_gps_raw_int_struct.lon = pvt.lon;
  mavlink_out_gps_raw_int_struct.alt = pvt.h;
  mavlink_out_gps_raw_int_struct.eph = pvt.pDOP;
  mavlink_out_gps_raw_int_struct.epv = UINT16_MAX;
  if (pvt.fixFlags & 1)
    mavlink_out_gps_raw_int_struct.fix_type = pvt.fixType;
  else
    mavlink_out_gps_raw_int_struct.fix_type = 0;
  mavlink_out_gps_raw_int_struct.satellites_visible = pvt.numSV;
  mavlink_out_gps_raw_int_struct.cog = pvt.hdg / 1000;
  mavlink_out_gps_raw_int_struct.vel = pvt.gSpeed;

  log_append();
}

/**
 *
 */
void gps_configure_mtk(void) {

  /* start on default baudrate */
  gps_ser_cfg.speed = GNSS_DEFAULT_BAUDRATE;
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
//void gps_configure_ubx(void) {
//
//  /* start on default baudrate */
//  gps_ser_cfg.speed = GPS_DEFAULT_BAUDRATE;
//  sdStart(&GPSSD, &gps_ser_cfg);
//
//  ubx_message_decimate("GLL", 0);
//  ubx_message_decimate("GLL", 0);// hack: write message twice for port buffer cleaning
//  ubx_message_decimate("GSV", 0);
//  ubx_message_decimate("GSA", 0);
//  ubx_message_decimate("VTG", 0);
//
//  ubx_solution_period(200);
//}

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
    result->msec       = rmc.msec;
    result->fix        = gga.fix;
    result->fresh      = true; // this line must be at the very end
  }
}

/**
int tm_sec       seconds after minute [0-61] (61 allows for 2 leap-seconds)
int tm_min       minutes after hour [0-59]
int tm_hour      hours after midnight [0-23]
int tm_mday      day of the month [1-31]
int tm_mon       month of year [0-11]
int tm_year      current year-1900
int tm_wday      days since Sunday [0-6]
int tm_yday      days since January 1st [0-365]
int tm_isdst     daylight savings indicator (1 = yes, 0 = no, -1 = unknown)
 */
static void pvt2time(const ubx_nav_pvt &pvt, struct tm *time) {
  memset(time, 0, sizeof(struct tm));

  time->tm_year = pvt.year - 1900;
  time->tm_mon  = pvt.month - 1;
  time->tm_mday = pvt.day;
  time->tm_hour = pvt.hour;
  time->tm_min  = pvt.min;
  time->tm_sec  = pvt.sec;
}

/**
 *
 */
static void gnss_unpack(const ubx_nav_pvt &pvt, gnss_data_t *result) {

  if (false == result->fresh) {
    result->altitude   = pvt.h / 1000.0;
    result->latitude   = pvt.lat;
    result->latitude   /= DEG_TO_MAVLINK;
    result->longitude  = pvt.lon;
    result->longitude  /= DEG_TO_MAVLINK;
    result->v[0] = pvt.velN / 100.0;
    result->v[1] = pvt.velE / 100.0;
    result->v[2] = pvt.velD / 100.0;
    result->speed_type = speed_t::VECTOR_3D;
    pvt2time(pvt, &result->time);
    result->msec       = pvt.nano / 1000000;
    if (pvt.fixFlags & 1)
      result->fix      = pvt.fixType;
    else
      result->fix      = 0;
    result->fresh      = true; // this line must be at the very end
  }
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
static ubx_ack_t ubx_wait_ack(SerialDriver *ubx_sd, ubx_msg_t type, systime_t timeout) {
  msg_t byte;
  systime_t start = chVTGetSystemTimeX();
  systime_t end = start + timeout;
  ubx_msg_t status;
  ubx_ack_nack ack_nack;
  ubx_ack_ack ack_ack;
  ubx_ack_t ret = ubx_ack_t::NONE;

  while (chVTIsSystemTimeWithinX(start, end)) {
    byte = sdGetTimeout(ubx_sd, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      status = ubx_parser.collect(byte);

      switch(status) {
      case ubx_msg_t::EMPTY:
        break;
      case ubx_msg_t::ACK_ACK:
        ubx_parser.unpack(ack_ack);
        if (ack_ack.msg_type == type) {
          ret = ubx_ack_t::ACK;
          goto EXIT;
        }
        break;
      case ubx_msg_t::ACK_NACK:
        ubx_parser.unpack(ack_nack);
        if (ack_nack.msg_type == type) {
          ret = ubx_ack_t::NACK;
          goto EXIT;
        }
        break;
      default:
        ubx_parser.drop(); // it is essential to drop unneded message
        break;
      }
    }
  }

EXIT:
  return ret;
}

/**
 * @brief   set solution period
 */
static void ubx_fix_period(uint16_t msec) {
  ubx_cfg_rate msg;
  uint8_t buf[sizeof(msg) + UBX_OVERHEAD_TOTAL];
  size_t len;
  ubx_ack_t ack;

  msg.measRate = msec;
  msg.navRate = 1;
  msg.timeRef = 0;

  len = ubx_parser.pack(msg, ubx_msg_t::CFG_RATE, buf, sizeof(buf));
  osalDbgCheck(len > 0 && len <= sizeof(buf));
  sdWrite(&GPSSD, buf, len);
  ack = ubx_wait_ack(&GPSSD, ubx_msg_t::CFG_RATE, S2ST(3));
  osalDbgCheck(ack == ubx_ack_t::ACK);
}

/**
 * @brief   set solution period
 */
static void ubx_port_settings(void) {
  ubx_cfg_prt msg;
  uint8_t buf[sizeof(msg) + UBX_OVERHEAD_TOTAL];
  size_t len;

  msg.portID = 1;
  msg.txready = 0;
  msg.mode = (0b11 << 6) | (0b100 << 9);
  msg.baudrate = GNSS_HI_BAUDRATE;
  msg.inProtoMask = 1;
  //msg.outProtoMask = 0b11; // nmea + ubx
  msg.outProtoMask = 0b1; // ubx only
  msg.flags = 0;

  len = ubx_parser.pack(msg, ubx_msg_t::CFG_PRT, buf, sizeof(buf));
  osalDbgCheck(len > 0 && len <= sizeof(buf));
  sdWrite(&GPSSD, buf, len);
  osalThreadSleepMilliseconds(100);
}

/**
 * @brief   set solution period
 */
static void ubx_filter_profile(uint32_t dyn_model) {
  ubx_ack_t ack;
  ubx_cfg_nav5 msg;
  uint8_t buf[sizeof(msg) + UBX_OVERHEAD_TOTAL];
  size_t len;

  /* protect from setting of reserved value */
  if (dyn_model == 1)
    dyn_model = 0;

  msg.dynModel = dyn_model;
  msg.fixMode = 2;
  msg.mask = 0b101;

  len = ubx_parser.pack(msg, ubx_msg_t::CFG_NAV5, buf, sizeof(buf));
  osalDbgCheck(len > 0 && len <= sizeof(buf));
  sdWrite(&GPSSD, buf, len);
  ack = ubx_wait_ack(&GPSSD, ubx_msg_t::CFG_NAV5, S2ST(1));
  osalDbgCheck(ack == ubx_ack_t::ACK);
}

/**
 * @brief   set solution period
 */
static void ubx_message_rate(void) {
  ubx_ack_t ack;
  ubx_cfg_msg msg;
  uint8_t buf[sizeof(msg) + UBX_OVERHEAD_TOTAL];
  size_t len;

  msg.rate = 1;
  msg.msg_type = ubx_msg_t::NAV_PVT;

  len = ubx_parser.pack(msg, ubx_msg_t::CFG_MSG, buf, sizeof(buf));
  osalDbgCheck(len > 0 && len <= sizeof(buf));
  sdWrite(&GPSSD, buf, len);
  ack = ubx_wait_ack(&GPSSD, ubx_msg_t::CFG_MSG, S2ST(1));
  osalDbgCheck(ack == ubx_ack_t::ACK);
}

/**
 *
 */
void ubx_configure(uint32_t dyn_model, uint32_t fix_period) {

  /* start on default baudrate */
  gps_ser_cfg.speed = GNSS_DEFAULT_BAUDRATE;
  sdStart(&GPSSD, &gps_ser_cfg);

  ubx_port_settings();
  sdStop(&GPSSD);
  gps_ser_cfg.speed = GNSS_HI_BAUDRATE;
  sdStart(&GPSSD, &gps_ser_cfg);

  ubx_fix_period(fix_period);
  ubx_filter_profile(dyn_model);
  ubx_message_rate();
}

/**
 *
 */
void GNSSReceiver::update_settings(void) {

  if (dyn_model_cache != *dyn_model) {
    dyn_model_cache = *dyn_model;
    ubx_filter_profile(dyn_model_cache);
  }

  if (fix_period_cache != *fix_period) {
    fix_period_cache = *fix_period;
    ubx_fix_period(fix_period_cache);
  }
}

/**
 *
 */
__CCM__ static THD_WORKING_AREA(gnssRxThreadWA, 400);
THD_FUNCTION(GNSSReceiver::nmeaRxThread, arg) {
  chRegSetThreadName("GNSS_NMEA");
  GNSSReceiver *self = static_cast<GNSSReceiver *>(arg);
  msg_t byte;
  sentence_type_t status;
  nmea_gga_t gga;
  nmea_rmc_t rmc;
  uint16_t gga_msec = GGA_VOID;
  uint16_t rmc_msec = RMC_VOID;

  osalThreadSleepSeconds(2);
  //gps_configure_mtk();
  osalSysLock();
  self->dyn_model_cache = *self->dyn_model;
  self->fix_period_cache = *self->fix_period;
  osalSysUnlock();
  ubx_configure(self->dyn_model_cache, self->fix_period_cache);

  while (!chThdShouldTerminateX()) {
    self->update_settings();

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
void GNSSReceiver::pvtdispatch(const ubx_nav_pvt &pvt) {
  acquire();
  for (size_t i=0; i<ArrayLen(this->spamlist); i++) {
    if (nullptr != this->spamlist[i]) {
      gnss_unpack(pvt, this->spamlist[i]);
    }
  }
  release();

  if (((pvt.fixType == 3) || (pvt.fixType == 4)) && (pvt.fixFlags & 1)) {
    event_gps.broadcastFlags(EVMSK_GNSS_FRESH_VALID);
  }
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
  ubx_nav_pvt pvt;

  osalThreadSleepSeconds(2);

  osalSysLock();
  self->dyn_model_cache  = *self->dyn_model;
  self->fix_period_cache = *self->fix_period;
  osalSysUnlock();
  ubx_configure(self->dyn_model_cache, self->fix_period_cache);

  while (!chThdShouldTerminateX()) {
    self->update_settings();
    byte = sdGetTimeout(&GPSSD, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      status = ubx_parser.collect(byte);

      switch(status) {
      case ubx_msg_t::EMPTY:
        break;
      case ubx_msg_t::NAV_PVT:
        ubx_parser.unpack(pvt);
        gps2mavlink(pvt);
        self->pvtdispatch(pvt);
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

  param_registry.valueSearch("GNSS_dyn_model",  &dyn_model);
  param_registry.valueSearch("GNSS_fix_period", &fix_period);

  worker = chThdCreateStatic(gnssRxThreadWA, sizeof(gnssRxThreadWA),
                      GPSPRIO, ubxRxThread, this);
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
