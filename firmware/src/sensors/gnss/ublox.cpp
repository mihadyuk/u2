#pragma GCC optimize "-O2"

#include "main.h"

#include "ubx_proto.hpp"
#include "ublox.hpp"
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
extern mavlink_gps_raw_int_t    mavlink_out_gps_raw_int_struct;

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
void uBlox::pvt2mavlink(const ubx_nav_pvt_payload &pvt) {

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
  mavlink_out_gps_raw_int_struct.vel = pvt.gSpeed / 10;

  log_append(&mavlink_out_gps_raw_int_struct);
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
static void pvt2time(const ubx_nav_pvt_payload &pvt, struct tm *time) {
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
ublox_ack_t uBlox::wait_ack(ubx_msg_t type, systime_t timeout) {
  msg_t byte;
  systime_t start = chVTGetSystemTimeX();
  systime_t end = start + timeout;
  ubx_msg_t status;
  ubx_ack_nack ack_nack;
  ubx_ack_ack ack_ack;
  ublox_ack_t ret = ublox_ack_t::NONE;

  while (chVTIsSystemTimeWithinX(start, end)) {
    byte = sdGetTimeout(this->sdp, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      status = ubx_parser.collect(byte);

      switch(status) {
      case ubx_msg_t::EMPTY:
        break;
      case ubx_msg_t::ACK_ACK:
        ubx_parser.unpack(ack_ack);
        if (ack_ack.data.acked_msg == type) {
          ret = ublox_ack_t::ACK;
          goto EXIT;
        }
        break;
      case ubx_msg_t::ACK_NACK:
        ubx_parser.unpack(ack_nack);
        if (ack_nack.data.nacked_msg == type) {
          ret = ublox_ack_t::NACK;
          goto EXIT;
        }
        break;
      default:
        ubx_parser.drop(); // it is very important to drop unneeded message
        break;
      }
    }
  }

EXIT:
  return ret;
}

/**
 * @brief     Write command and wait (if needed) confirmation.
 * @note      Set timeout to 0 if confirm not needed
 */
template <typename T>
void uBlox::write_with_confirm(const T &msg, systime_t timeout) {

  uint8_t buf[sizeof(msg) + UBX_OVERHEAD_TOTAL];
  size_t len;
  ublox_ack_t ack;

  len = ubx_parser.pack(msg, buf, sizeof(buf));
  osalDbgCheck(len > 0 && len <= sizeof(buf));
  sdWrite(this->sdp, buf, len);
  if (0 != timeout) {
    ack = wait_ack(msg.rtti, timeout);
    osalDbgCheck(ack == ublox_ack_t::ACK);
  }
}

/**
 * @brief   set solution period
 */
void uBlox::set_fix_period(uint16_t msec) {
  ubx_cfg_rate msg;

  msg.data.measRate = msec;
  msg.data.navRate = 1;
  msg.data.timeRef = 0;

  this->write_with_confirm(msg, S2ST(1));
}

/**
 * @brief   set solution period
 */
void uBlox::set_port(void) {
  ubx_cfg_prt msg;

  msg.data.portID = 1;
  msg.data.txready = 0;
  msg.data.mode = (0b11 << 6) | (0b100 << 9);
  msg.data.baudrate = GNSS_HI_BAUDRATE;
  msg.data.inProtoMask = 1;
  //msg.outProtoMask = 0b11; // nmea + ubx
  msg.data.outProtoMask = 0b1; // ubx only
  msg.data.flags = 0;

  write_with_confirm(msg, 0);
  osalThreadSleepMilliseconds(100);
}

/**
 * @brief   set solution period
 */
void uBlox::set_dyn_model(uint32_t dyn_model) {
  ubx_cfg_nav5 msg;

  /* protect from setting of reserved value */
  if (dyn_model == 1)
    dyn_model = 0;

  msg.data.dynModel = dyn_model;
  msg.data.fixMode = 2;
  msg.data.mask = 0b101;

  write_with_confirm(msg, S2ST(1));
}

/**
 * @brief   set solution period
 */
void uBlox::set_message_rate(void) {
  ubx_cfg_msg msg;

  msg.data.rate = 1;
  msg.data.msg_wanted = ubx_msg_t::NAV_PVT;

  write_with_confirm(msg, S2ST(1));
}

/**
 *
 */
void uBlox::configure(uint32_t dyn_model, uint32_t fix_period) {

  gps_ser_cfg.speed = GNSS_DEFAULT_BAUDRATE;
  sdStart(this->sdp, &gps_ser_cfg);
  set_port();
  sdStop(this->sdp);
  gps_ser_cfg.speed = GNSS_HI_BAUDRATE;
  sdStart(this->sdp, &gps_ser_cfg);

  set_fix_period(fix_period);
  set_dyn_model(dyn_model);
  set_message_rate();
}

/**
 *
 */
void uBlox::update_settings(void) {

  if (dyn_model_cache != *dyn_model) {
    dyn_model_cache = *dyn_model;
    set_dyn_model(dyn_model_cache);
  }

  if (fix_period_cache != *fix_period) {
    fix_period_cache = *fix_period;
    set_fix_period(fix_period_cache);
  }
}

/**
 *
 */
static void pvt2gnss(const ubx_nav_pvt_payload &pvt, gnss_data_t *result) {

  result->altitude    = pvt.h / 1000.0;
  result->latitude    = pvt.lat;
  result->latitude    /= DEG_TO_MAVLINK;
  result->longitude   = pvt.lon;
  result->longitude   /= DEG_TO_MAVLINK;
  result->v[0]        = pvt.velN / 1000.0;
  result->v[1]        = pvt.velE / 1000.0;
  result->v[2]        = pvt.velD / 1000.0;
  result->speed       = pvt.gSpeed / 1000.0;
  result->course      = pvt.hdg * 1e-5;
  result->speed_type  = speed_t::BOTH;
  pvt2time(pvt, &result->time);
  result->msec        = pvt.nano / 1000000;
  if (pvt.fixFlags & 1)
    result->fix       = pvt.fixType;
  else
    result->fix       = 0;
  result->fresh       = true;
}

/**
 *
 */
void uBlox::pvtdispatch(const ubx_nav_pvt_payload &pvt) {

  acquire();
  pvt2gnss(pvt, &this->cache);
  cache.fresh = false;

  for (size_t i=0; i<ArrayLen(this->spamlist); i++) {
    gnss_data_t *p = this->spamlist[i];
    if ((nullptr != p) && (false == p->fresh)) {
      memcpy(p, &this->cache, sizeof(this->cache));
      p->fresh = true; // this line must be at the very end for atomicity
    }
  }
  release();

  if (((pvt.fixType == 3) || (pvt.fixType == 4)) && (pvt.fixFlags & 1)) {
    event_gnss.broadcastFlags(EVMSK_GNSS_FRESH_VALID);
  }
}

/**
 *
 */
THD_FUNCTION(uBlox::ubxRxThread, arg) {
  chRegSetThreadName("GNSS_UBX");
  uBlox *self = static_cast<uBlox *>(arg);
  msg_t byte;
  ubx_msg_t status;
  ubx_nav_pvt pvt;

  osalThreadSleepSeconds(2);

  osalSysLock();
  self->dyn_model_cache  = *self->dyn_model;
  self->fix_period_cache = *self->fix_period;
  osalSysUnlock();
  self->configure(self->dyn_model_cache, self->fix_period_cache);

  while (!chThdShouldTerminateX()) {
    self->update_settings();
    byte = sdGetTimeout(self->sdp, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      status = self->ubx_parser.collect(byte);

      switch(status) {
      case ubx_msg_t::EMPTY:
        break;
      case ubx_msg_t::NAV_PVT:
        self->ubx_parser.unpack(pvt);
        self->pvt2mavlink(pvt.data);
        self->pvtdispatch(pvt.data);
        break;
      default:
        self->ubx_parser.drop(); // it is essential to drop unneeded message
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
uBlox::uBlox(SerialDriver *sdp) : GNSSReceiver(sdp) {
  return;
}

/**
 *
 */
void uBlox::start(void) {

  param_registry.valueSearch("GNSS_dyn_model",  &dyn_model);
  param_registry.valueSearch("GNSS_fix_period", &fix_period);

  worker = chThdCreateStatic(gnssRxThreadWA, sizeof(gnssRxThreadWA),
                      GPSPRIO, ubxRxThread, this);
  osalDbgCheck(nullptr != worker);
  ready = true;
}
