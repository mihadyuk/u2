#pragma GCC optimize "-O2"

#include <ctime>
#include <cmath>
#include <cstring>
#include <cstdlib>

#include "main.h"
#include "proto_nmea.hpp"
#include "pads.h"
#include "array_len.hpp"

using namespace gnss;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define GPS_MIN_MSG_LEN       12

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
/**
 *
 */
static const char * const test_sentence_array[] = {
    "$PMTK001,300,3*33",
    "$PMTK251,57600*2C\r\n"
    "$PMTK300,200,0,0,0,0*2F\r\n",
    "$PMTK300,250,0,0,0,0*2A\r\n",
    "$PMTK300,500,0,0,0,0*28\r\n",

    "$GPRMC,162254.00,A,3723.02837,N,12159.39853,W,0.820,188.36,110706,,,A*74",
    "$GPVTG,188.36,T,,M,0.820,N,1.519,K,A*3F",
    "$GPGGA,162254.00,3723.02837,N,12159.39853,W,1,03,2.36,525.6,M,-25.6,M,,*65",
    "$GPGSA,A,2,25,01,22,,,,,,,,,,2.56,2.36,1.00*02",
    "$GPGSV,4,1,14,25,15,175,30,14,80,041,,19,38,259,14,01,52,223,18*76",
    "$GPGSV,4,2,14,18,16,079,,11,19,312,,14,80,041,,21,04,135,25*7D",
    "$GPGSV,4,3,14,15,27,134,18,03,25,222,,22,51,057,16,09,07,036,*79",
    "$GPGSV,4,4,14,07,01,181,,15,25,135,*76",
    "$GPGLL,3723.02837,N,12159.39853,W,162254.00,A,A*7C"
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
bool ProtoNmea::_autotest(const char *sentence) const {
  const char *sump = strstr(sentence, "*");
  size_t len = sump - sentence;

  uint8_t ref_sum = checksumFromStr(sump + 1);
  uint8_t sum = checksum((uint8_t *)sentence + 1, len - 1);
  if (sum != ref_sum)
    return OSAL_FAILED;

  char str_sum[2];
  checksum2str(sum, str_sum);
  if (0 != memcmp(sump+1, str_sum, 2))
    return OSAL_FAILED;

  return OSAL_SUCCESS;
}

/**
 *
 */
bool ProtoNmea::checksum_autotest(void) const {
  for (size_t i=0; i<ArrayLen(test_sentence_array); i++) {
    if (OSAL_FAILED == _autotest(test_sentence_array[i]))
      return OSAL_FAILED;
  }

  return OSAL_SUCCESS;
}

/**
 * Выковыривает дату и время из RMC
 *
 * timp - указатель на структуру с временем
 * buft - указатель на строку с временем
 * bufd - указатель на строку с датой
 *
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
static uint16_t get_time(struct tm *timp, const char *buft) {

  if (nullptr != timp) {
    timp->tm_hour = 10 * (buft[0] - '0') + (buft[1] - '0');
    timp->tm_min  = 10 * (buft[2] - '0') + (buft[3] - '0');
    timp->tm_sec  = 10 * (buft[4] - '0') + (buft[5] - '0');
  }

  /* fractional part */
  if ('.' == buft[6]) { // time stamp has decimal part
    size_t L = strlen(&buft[7]);
    uint32_t mul = 1000;
    for (size_t i=0; i<L; i++) {
      mul /= 10;
    }
    return atoi(&buft[7]) * mul;
  }
  else {
    return 0;
  }
}

static void get_date(struct tm *timp, const char *bufd) {
  timp->tm_mday = 10 * (bufd[0] - '0') + (bufd[1] - '0');
  timp->tm_mon  = 10 * (bufd[2] - '0') + (bufd[3] - '0') - 1;
  timp->tm_year = 10 * (bufd[4] - '0') + (bufd[5] - '0') + 2000 - 1900;
}

/**
 *
 */
static uint8_t _from_hex(uint8_t a){
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

/**
 *
 */
uint8_t ProtoNmea::checksumFromStr(const char *str) const {
  return (_from_hex(str[0]) << 4) | _from_hex(str[1]);
}

/**
 *
 */
static uint8_t _to_hex(uint8_t u8) {
  if (u8 < 10)
    return u8 + '0';
  else
    return u8 - 10 + 'A';
}

/**
 *
 */
void ProtoNmea::checksum2str(uint8_t sum, char *str) const {

  str[0] = _to_hex(sum >> 4);
  str[1] = _to_hex(sum & 0b1111);
}

/**
 *
 */
sentence_type_t ProtoNmea::get_name(const char *name) const {

  if ((0 == strncmp("GNGGA", name, 5)) || (0 == strncmp("GPGGA", name, 5)))
    return sentence_type_t::GGA;
  else if ((0 == strncmp("GNRMC", name, 5)) || (0 == strncmp("GPRMC", name, 5)))
    return sentence_type_t::RMC;
  else if (0 == strncmp("GLGSV", name, 5))
    return sentence_type_t::GLGSV;
  else if (0 == strncmp("GPGSV", name, 5))
    return sentence_type_t::GPGSV;
  else
    return sentence_type_t::UNKNOWN;
}

/**
 *
 */
sentence_type_t ProtoNmea::validate_sentence(void) const {
  uint8_t sum = 0;

  if (tip < GPS_MIN_MSG_LEN)
    return sentence_type_t::EMPTY;

  if ('$' != buf[0])
    return sentence_type_t::EMPTY;

  if ('\n' != buf[tip-1])
    return sentence_type_t::EMPTY;

  if ('\r' != buf[tip-2])
    return sentence_type_t::EMPTY;

  if ('*' != buf[tip-5])
    return sentence_type_t::EMPTY;

  /* now verify checksum */
  sum = checksum(&buf[1], tip-5-1);
  if (sum != checksumFromStr((const char*)&buf[tip-4]))
    return sentence_type_t::EMPTY;
  else
    return get_name((char *)&buf[1]);
}

/**
 *
 */
uint8_t ProtoNmea::checksum(const uint8_t *data, size_t len) const {
  uint8_t sum = 0;

  for (size_t i=0; i<len; i++) {
    sum ^= data[i];
  }

  return sum;
}

/**
 *
 */
size_t ProtoNmea::tokens_available(void) const {
  return this->maptip - 1;
}

/**
 *
 */
const char* ProtoNmea::token(char *result, size_t N) const {

  const size_t len = token_map[N+1] - (1 + token_map[N]);
  memset(result, 0, GPS_MAX_TOKEN_LEN);

  if ((len > 0) && (len < GPS_MAX_TOKEN_LEN)) {
    const char *src = (char *)&buf[token_map[N] + 1];
    memcpy(result, src, len);
  }

  return result;
}

/**
 *
 */
static double degrees_sign(const char *sign) {
  if (('S' == *sign) || ('W' == *sign))
    return -1;
  else
    return 1;
}

/**
 *
 */
static double gps2deg(double gps) {
  double deg;

  gps /= 100;
  deg = floor(gps);

  return deg + (gps - deg) * static_cast<double>(1.66666666666666666);
}

/**
 *
 */
static float knots2mps(float knots) {
  return knots * 0.514444444;
}

/**
 *
 */
void ProtoNmea::reset_collector(void) {
  tip = 0;
  maptip = 0;
  memset(this->buf, 0, sizeof(this->buf));
  memset(this->token_map, 0, sizeof(this->token_map));
  state = nmea_collect_state_t::START;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
ProtoNmea::ProtoNmea(void) :
tip(0),
maptip(0),
state(nmea_collect_state_t::START)
{
  memset(this->buf, 0, sizeof(this->buf));
  memset(this->token_map, 0, sizeof(this->token_map));

  osalDbgAssert(OSAL_SUCCESS == checksum_autotest(),
                "NMEA: checksum autotest failed");
}

/**
 *
 */
sentence_type_t ProtoNmea::collect(uint8_t c) {
  sentence_type_t ret = sentence_type_t::EMPTY;

  /* prevent buffer overflow */
  if ((tip >= GPS_MSG_LEN) || (maptip >= GPS_TOKEN_MAP_LEN)) {
    reset_collector();
  }

  /* NMEA message contains only printable ASCII characters */
  if (('\r' != c) && ('\n' != c)) {
    if ((c < 0x20) || (c > 0x7E)) {
      reset_collector();
    }
  }

  switch (state) {
  case nmea_collect_state_t::START:
    if ('$' == c) {
      reset_collector();
      buf[0] = c;
      tip = 1;
      state = nmea_collect_state_t::DATA;
    }
    break;

  case nmea_collect_state_t::DATA:
    buf[tip] = c;
    if ((',' == c) || ('*' == c)) {
      token_map[maptip] = tip;
      maptip++;
    }
    tip++;
    if (tip >= sizeof(buf) - 4)
      reset_collector();
    if ('*' == c)
      state = nmea_collect_state_t::CHECKSUM1;
    break;

  case nmea_collect_state_t::CHECKSUM1:
    buf[tip] = c;
    tip++;
    state = nmea_collect_state_t::CHECKSUM2;
    break;

  case nmea_collect_state_t::CHECKSUM2:
    buf[tip] = c;
    tip++;
    state = nmea_collect_state_t::EOL_CR;
    break;

  case nmea_collect_state_t::EOL_CR:
    if ('\r' == c) {
      buf[tip] = c;
      tip++;
      state = nmea_collect_state_t::EOL_LF;
    }
    else
      reset_collector();
    break;

  case nmea_collect_state_t::EOL_LF:
    if ('\n' == c) {
      buf[tip] = c;
      tip++;
      ret = validate_sentence();
      if (sentence_type_t::EMPTY == ret)
        reset_collector();
      state = nmea_collect_state_t::START;
    }
    else
      reset_collector();
    break;

  default:
    osalSysHalt("unhandled case");
    break;
  }

  return ret;
}

/**
 *
 */
void ProtoNmea::unpack(nmea_gga_t &result) const {
  char tmp[GPS_MAX_TOKEN_LEN];
  double c;

  result.msec        = get_time(&result.time, token(tmp, 0));
  c = copysign(atof(token(tmp, 1)), degrees_sign(token(tmp, 2)));
  result.latitude    = gps2deg(c);
  c = copysign(atof(token(tmp, 3)), degrees_sign(token(tmp, 4)));
  result.longitude   = gps2deg(c);
  result.fix         = atol(token(tmp, 5));
  result.satellites  = atol(token(tmp, 6));
  result.hdop        = atof(token(tmp, 7));
  result.altitude    = atof(token(tmp, 8));
  result.geoid       = atof(token(tmp, 10));
}

/**
 *
 */
void ProtoNmea::unpack(nmea_gsv_t &result) const {
  char tmp[GPS_MAX_TOKEN_LEN];

  result.total        = atoi(token(tmp, 0));
  result.current      = atoi(token(tmp, 1));
  result.sat_visible  = atoi(token(tmp, 2));

  // silently prevent overflow
  size_t satcnt = (tokens_available() - 1) / 4;
  if (satcnt > ArrayLen(result.sat))
    satcnt = ArrayLen(result.sat);

  /* reset to known state */
  for (size_t i=0; i<ArrayLen(result.sat); i++)
    memset(&result.sat, 0, sizeof(result.sat));

  /* fill apropriate fields */
  for (size_t i=0; i<satcnt; i++) {
    nmea_gsv_satellite_t &sat = result.sat[i];
    sat.id         = atoi(token(tmp, i*4 + 3));
    sat.elevation  = atoi(token(tmp, i*4 + 4));
    sat.azimuth    = atoi(token(tmp, i*4 + 5));
    sat.snr        = atoi(token(tmp, i*4 + 6));
  }
}

/**
 *
 */
void ProtoNmea::unpack(nmea_rmc_t &result) const {
  char tmp[GPS_MAX_TOKEN_LEN];

  result.msec   = get_time(&result.time, token(tmp, 0));
  result.speed  = knots2mps(atof(token(tmp, 6)));
  result.course = atof(token(tmp, 7));
  get_date(&result.time, token(tmp, 8));
}

/**
 * @brief   Fills checksum field and inserts CR/LF
 * @pre     Initial string must be started from '$' ended with '*'
 */
void ProtoNmea::seal(char *msg) {
  char *sump = strstr(msg, "*");
  osalDbgCheck(nullptr != sump);

  size_t len = sump - msg;
  uint8_t sum = checksum((uint8_t *)msg + 1, len - 1);

  checksum2str(sum, sump+1);
  sump[3] = '\r';
  sump[4] = '\n';
}

