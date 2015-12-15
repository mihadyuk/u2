#pragma GCC optimize "-O2"

#include "main.h"

#include "proto_ubx.hpp"
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
extern mavlink_gps_status_t     mavlink_out_gps_status_struct;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static const double UBX_TO_DEG = 1e-7;

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
static uint16_t check_period(uint16_t period) {
  if (period <= 200)
    return 200;
  else if (period <= 400)
    return 400;
  else if (period <= 500)
    return 500;
  else
    return 1000;
}

/**
 *
 */
static uint8_t uxb_azim_to_mavlink(int16_t azim) {
  float a = (azim + 180) * 0.708333f;
  return roundf(a);
}

/**
 *
 */
void uBlox::sat2mavlink(void) {

  memset(&mavlink_out_gps_status_struct, 0, sizeof(mavlink_out_gps_status_struct));

  size_t cnt;
  if (this->sat.payload.numSvs > ArrayLen(mavlink_out_gps_status_struct.satellite_azimuth))
    cnt = ArrayLen(mavlink_out_gps_status_struct.satellite_azimuth);
  else
    cnt = this->sat.payload.numSvs;

  mavlink_out_gps_status_struct.satellites_visible = this->sat.payload.numSvs;

  for (size_t i=0; i<cnt; i++) {
    mavlink_out_gps_status_struct.satellite_azimuth[i]   = uxb_azim_to_mavlink(this->sat.payload.svs[i].azim);
    mavlink_out_gps_status_struct.satellite_elevation[i] = this->sat.payload.svs[i].elev;
    mavlink_out_gps_status_struct.satellite_prn[i]       = this->sat.payload.svs[i].svID;
    mavlink_out_gps_status_struct.satellite_snr[i]       = this->sat.payload.svs[i].cno;
    mavlink_out_gps_status_struct.satellite_used[i]      = (this->sat.payload.svs[i].flags >> 3) & 1;
  }
}

/**
 *
 */
void uBlox::pvt_dop2mavlink(void) {

  const ubx_nav_pvt_payload &pvt = this->pvt.payload;
  const ubx_nav_dop_payload &dop = this->dop.payload;

  mavlink_out_gps_raw_int_struct.time_usec = TimeKeeper::utc();
  mavlink_out_gps_raw_int_struct.lat = pvt.lat;
  mavlink_out_gps_raw_int_struct.lon = pvt.lon;
  mavlink_out_gps_raw_int_struct.alt = pvt.h;
  mavlink_out_gps_raw_int_struct.eph = dop.pDOP;
  mavlink_out_gps_raw_int_struct.epv = dop.vDOP;
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
 * @note    Do NOT forget to drop received message if any.
 */
ubx_msg_t uBlox::wait_any_timeout(const ubx_msg_t *type_list,
                            size_t list_len, systime_t timeout) {
  msg_t byte;
  systime_t start = chVTGetSystemTimeX();
  systime_t end = start + timeout;
  ubx_msg_t status = ubx_msg_t::EMPTY;

  for (size_t i=0; i<list_len; i++) {
    osalDbgAssert(type_list[i] != ubx_msg_t::EMPTY, "Empty type forbidden");
  }

  while (chVTIsSystemTimeWithinX(start, end)) {
    byte = sdGetTimeout(this->sdp, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      status = parser.collect(byte);
      if (ubx_msg_t::EMPTY != status) {
        for (size_t i=0; i<list_len; i++) {
          if (type_list[i] == status) {
            return status;
          }
        }
        parser.drop(); // it is very important to drop unneeded message
      }
    }
  }

  return ubx_msg_t::EMPTY;
}

/**
 *
 */
ubx_msg_t uBlox::wait_one_timeout(ubx_msg_t type, systime_t timeout) {
  return wait_any_timeout(&type, 1, timeout);
}

/**
 *
 */
ublox_ack_t uBlox::wait_ack(ubx_msg_t type, systime_t timeout) {

  ubx_msg_t status = ubx_msg_t::EMPTY;
  ubx_ack_nack ack_nack;
  ubx_ack_ack ack_ack;
  ublox_ack_t ret = ublox_ack_t::NONE;

  ubx_msg_t list[2] = {ubx_msg_t::ACK_ACK, ubx_msg_t::ACK_NACK};
  status = wait_any_timeout(list, 2, timeout);

  switch(status) {
  case ubx_msg_t::EMPTY:
    ret = ublox_ack_t::NONE;
    goto EXIT;
    break;
  case ubx_msg_t::ACK_ACK:
    parser.unpack(ack_ack);
    if (ack_ack.payload.acked_msg == type) {
      ret = ublox_ack_t::ACK;
      goto EXIT;
    }
    break;
  case ubx_msg_t::ACK_NACK:
    parser.unpack(ack_nack);
    if (ack_nack.payload.nacked_msg == type) {
      ret = ublox_ack_t::NACK;
      goto EXIT;
    }
    break;
  default:
    osalSysHalt("Logic broken. This message unexpected here.");
    break;
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

  len = parser.pack(msg, buf, sizeof(buf));
  osalDbgCheck(len > 0 && len <= sizeof(buf));
  sdWrite(this->sdp, buf, len);
  if (0 != timeout) {
    ack = wait_ack(msg.rtti, timeout);
    if (ack == ublox_ack_t::ACK)
      this->nack_cnt++;
    if (ack == ublox_ack_t::NONE)
      this->no_answer_cnt++;
  }
}

/**
 * @brief   set solution period
 */
void uBlox::set_fix_period(uint16_t msec) {
  ubx_cfg_rate msg;

  msg.payload.measRate = check_period(msec);
  msg.payload.navRate = 1;
  msg.payload.timeRef = 0;

  this->write_with_confirm(msg, S2ST(1));
}

/**
 * @brief   set communication options
 */
void uBlox::set_port(void) {
  ubx_cfg_prt msg;

  msg.payload.portID = 1;
  msg.payload.txready = 0;
  msg.payload.mode = (0b11 << 6) | (0b100 << 9);
  msg.payload.baudrate = this->working_baudrate;
  msg.payload.inProtoMask = 1;
  //msg.payload.outProtoMask = 0b11; // nmea + ubx
  msg.payload.outProtoMask = 0b1; // ubx only
  msg.payload.flags = 0;

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

  msg.payload.dynModel = dyn_model;
  msg.payload.fixMode = 2;
  msg.payload.mask = 0b101;

  write_with_confirm(msg, S2ST(1));
}

/**
 * @brief   set solution messages drop rate
 */
void uBlox::set_message_rate(void) {
  ubx_cfg_msg msg;

  msg.payload.rate = 1;
  msg.payload.msg_wanted = ubx_msg_t::NAV_PVT;
  write_with_confirm(msg, S2ST(1));

  msg.payload.rate = 1;
  msg.payload.msg_wanted = ubx_msg_t::NAV_DOP;
  write_with_confirm(msg, S2ST(1));

  msg.payload.rate = 1;
  msg.payload.msg_wanted = ubx_msg_t::NAV_SAT;
  write_with_confirm(msg, S2ST(1));
}

/**
 *
 */
bool uBlox::device_alive(systime_t timeout) {
  uint8_t buf[UBX_OVERHEAD_TOTAL];
  size_t len;
  ubx_msg_t recvd = ubx_msg_t::EMPTY;
  bool ret = false;
  ubx_mon_ver<0> version;

  len = parser.packPollRequest(ubx_msg_t::MON_VER, buf, sizeof(buf));
  osalDbgCheck(len > 0 && len <= sizeof(buf));

  sdWrite(this->sdp, buf, len);
  recvd = wait_one_timeout(ubx_msg_t::MON_VER, timeout);
  if (ubx_msg_t::MON_VER == recvd) {
    this->parser.unpack(version);
    ret = true;
  }
  else {
    ret = false;
  }

  return ret;
}

/**
 *
 */
void uBlox::get_version(void) {

  uint8_t buf[UBX_OVERHEAD_TOTAL];
  size_t len;
  ubx_msg_t recvd = ubx_msg_t::EMPTY;

  len = parser.packPollRequest(ubx_msg_t::MON_VER, buf, sizeof(buf));
  osalDbgCheck(len > 0 && len <= sizeof(buf));

  sdWrite(this->sdp, buf, len);
  recvd = wait_one_timeout(ubx_msg_t::MON_VER, S2ST(1));
  if (ubx_msg_t::MON_VER == recvd) {
    this->parser.unpack(this->ver);
  }
}

/**
 *
 */
void uBlox::configure(uint32_t dyn_model, uint32_t fix_period) {

  set_port();

  sdStop(this->sdp);
  gps_serial_cfg.speed = this->working_baudrate;
  sdStart(this->sdp, &gps_serial_cfg);

  get_version();
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
static void pvt2gnss(const ubx_nav_pvt_payload &pvt,
                     const ubx_nav_dop_payload &dop,
                     float samplerate, gnss_data_t *result) {

  result->altitude    = pvt.h / 1000.0;
  result->latitude    = deg2rad(pvt.lat * UBX_TO_DEG);
  result->longitude   = deg2rad(pvt.lon * UBX_TO_DEG);
  result->v[0]        = pvt.velN / 1000.0;
  result->v[1]        = pvt.velE / 1000.0;
  result->v[2]        = pvt.velD / 1000.0;
  result->speed       = pvt.gSpeed / 1000.0;
  result->course      = deg2rad(pvt.hdg * 1e-5);
  result->speed_type  = speed_t::BOTH;
  pvt2time(pvt, &result->time);
  result->msec        = pvt.nano / 1000000;
  if (pvt.fixFlags & 1)
    result->fix       = pvt.fixType;
  else
    result->fix       = 0;
  result->samplerate  = samplerate;

  result->hdop        = dop.hDOP / 100.0;
  result->vdop        = dop.vDOP / 100.0;
  result->pdop        = dop.pDOP / 100.0;
}

/**
 *
 */
void uBlox::gnss_dispatch(void) {

  const ubx_nav_pvt_payload &pvt = this->pvt.payload;
  const ubx_nav_dop_payload &dop = this->dop.payload;
  gnss_data_t cache;

  acquire();
  pvt2gnss(pvt, dop, 1000.0 / *fix_period, &cache);

  for (size_t i=0; i<ArrayLen(this->spamlist); i++) {
    gnss_data_t *p = this->spamlist[i];
    if ((nullptr != p) && (false == p->fresh)) {
      memcpy(p, &cache, sizeof(cache));
      p->fresh = true;
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
static void dbg_print(BaseSequentialStream *sdp, ubx_nav_pvt &pvt) { /*
  uint32_t iTOW;    // ms GPS time of week
  uint16_t year;    // 1999..2099
  uint8_t  month;   // 1..12
  uint8_t  day;     // 1..31
  uint8_t  hour;    // 0..23
  uint8_t  min;     // 0..59
  uint8_t  sec;     // 0..60
  uint8_t  valid;   // Validity  flags
  uint32_t tAcc;    // ns Time accuracy estimate
  int32_t  nano;    // ns Fraction of second -1e9..1e9

  uint8_t  fixType;
  uint8_t  fixFlags;
  uint8_t  reserved;
  uint8_t  numSV;   // number of satellites used in solution

  int32_t  lon;     // deg 10e-7
  int32_t  lat;     // deg 10e-7
  int32_t  h;       // mm Height above ellipsoid
  int32_t  hMSL;    // mm
  uint32_t hAcc;    // mm Horizontal accuracy estimate
  uint32_t vAcc;    // mm Vertical accuracy estimate

  int32_t  velN;    // mm/s
  int32_t  velE;    // mm/s
  int32_t  velD;    // mm/s
  uint32_t gSpeed;  // mm/s Speed module (2D)
  int32_t  hdg;     // deg * 1e-5 Heading of motion
  uint32_t speedAcc;// mm/s Speed accuracy estimate
  uint32_t hdgAcc;  // deg * 1e-5 Coarse/heading accuracy estimate

  uint16_t pDOP;    // 0.01
  uint8_t  reserved2[6];
  int32_t  hedVeh;  // deg * 1e-5 Heading of vehicle
  uint8_t  reserved3[4]; */
  if (nullptr != sdp) {
    chprintf(sdp, "lat = %D",    pvt.payload.lat);
    chprintf(sdp, ", lon = %D",  pvt.payload.lon);
    chprintf(sdp, ", h = %D",    pvt.payload.h);
    chprintf(sdp, ", hMSL = %D", pvt.payload.hMSL);
    chprintf(sdp, ", hAcc = %D", pvt.payload.hAcc);
    chprintf(sdp, ", vAcc = %D", pvt.payload.vAcc);
    chprintf(sdp, "\r\n");
  }
}

#define ITOW_INVALID    0

/**
 *
 */
THD_FUNCTION(uBlox::ubxRxThread, arg) {
  chRegSetThreadName("GNSS_UBX");
  uBlox *self = static_cast<uBlox *>(arg);
  msg_t byte = MSG_TIMEOUT;
  ubx_msg_t status = ubx_msg_t::EMPTY;
  uint32_t iTOW_dop = ITOW_INVALID;
  uint32_t iTOW_pvt = ITOW_INVALID;

  /* wait until receiver boots up */
  osalThreadSleepSeconds(2);
  while (true) {
    if (chThdShouldTerminateX())
      goto EXIT;
    if (self->device_alive(MS2ST(500)))
      break;
  }

  /* configure receiver */
  osalSysLock();
  self->dyn_model_cache  = *self->dyn_model;
  self->fix_period_cache = *self->fix_period;
  osalSysUnlock();
  self->configure(self->dyn_model_cache, self->fix_period_cache);

  /* main loop */
  while (!chThdShouldTerminateX()) {
    self->update_settings();
    byte = sdGetTimeout(self->sdp, MS2ST(50));
    if (MSG_TIMEOUT != byte) {
      status = self->parser.collect(byte);
      switch(status) {
      case ubx_msg_t::EMPTY:
        break;
      case ubx_msg_t::NAV_PVT:
        self->parser.unpack(self->pvt);
        dbg_print((BaseSequentialStream *)self->sniff_sdp, self->pvt);
        iTOW_pvt = self->pvt.payload.iTOW;
        break;
      case ubx_msg_t::NAV_DOP:
        self->parser.unpack(self->dop);
        iTOW_dop = self->dop.payload.iTOW;
        break;
      case ubx_msg_t::NAV_SAT:
        self->parser.unpack(self->sat);
        self->sat2mavlink();
        break;

      default:
        self->parser.drop(); // it is essential to drop unneeded message
        break;
      }

      /* All data collected. Now we need to pass it to cosumers */
      if ((iTOW_dop == iTOW_pvt) && (ITOW_INVALID != iTOW_dop) && (ITOW_INVALID != iTOW_pvt)) {
        self->pvt_dop2mavlink();
        self->gnss_dispatch();
        iTOW_dop = ITOW_INVALID;
        iTOW_pvt = ITOW_INVALID;
      }
    }
  }

EXIT:
  chThdExit(MSG_OK);
}

/**
 *
 */
void uBlox::start_impl(void) {

  param_registry.valueSearch("GNSS_dyn_model",  &dyn_model);
  param_registry.valueSearch("GNSS_fix_period", &fix_period);

  worker = chThdCreateStatic(gnssRxThreadWA, sizeof(gnssRxThreadWA),
                             GPSPRIO, ubxRxThread, this);
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
uBlox::uBlox(SerialDriver *sdp, uint32_t start_baudrate, uint32_t working_baudrate) :
    GNSSReceiver(sdp, start_baudrate, working_baudrate) {
  return;
}

