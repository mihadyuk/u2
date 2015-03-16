#include <ctime>
#include <cmath>
#include <cstring>
#include <cstdlib>

#include "main.h"
#include "nmea.hpp"

using namespace gps;

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
/*
$GPGGA,115436.000,5354.713670,N,02725.690517,E,1,5,2.01,266.711,M,26.294,M,,*5D
$GPGGA,000103.037,,,,,0,0,,,M,,M,,*4E

$GPRMC,115436.000,A,5354.713670,N,02725.690517,E,0.20,210.43,010611,,,A*66
$GPRMC,115436.000,,,,,,0.20,210.43,010611,,,A*66
$GPRMC,115436.000,,,,,,,,,,,A*66
*/

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
static void get_time(struct tm *timp, bool *round, const char *buft) {
  timp->tm_hour = 10 * (buft[0] - '0') + (buft[1] - '0');
  timp->tm_min  = 10 * (buft[2] - '0') + (buft[3] - '0');
  timp->tm_sec  = 10 * (buft[4] - '0') + (buft[5] - '0');

  /* check fractional part */
  if (('.' == buft[6]) && ('0' == buft[7]) && ('0' == buft[8]))
    *round = true;
  else
    *round = false;
}

static void get_date(struct tm *timp, const char *bufd) {
  timp->tm_mday = 10 * (bufd[0] - '0') + (bufd[1] - '0');
  timp->tm_mon  = 10 * (bufd[2] - '0') + (bufd[3] - '0') - 1;
  timp->tm_year = 10 * (bufd[4] - '0') + (bufd[5] - '0') + 2000 - 1900;
}

/**
 *
 */
static uint8_t from_hex(uint8_t a){
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
collect_status_t NmeaParser::get_name(const char *name) {
  if (0 == strncmp("GPGGA", name, 5))
    return collect_status_t::GPGGA;
  else if (0 == strncmp("GPRMC", name, 5))
    return collect_status_t::GPRMC;
  else
    return collect_status_t::UNKNOWN;
}

/**
 *
 */
collect_status_t NmeaParser::validate_sentece(void) {
  uint8_t sum = 0;

  if (tip < GPS_MIN_MSG_LEN)
    return collect_status_t::EMPTY;

  if ('$' != buf[0])
    return collect_status_t::EMPTY;

  for (size_t i=1; i<6; i++) {
    if ((buf[i] < 'A') || (buf[i] > 'Z'))   /* letters from 'A' to 'Z' */
      return collect_status_t::EMPTY;
  }

  if (',' != buf[6])                        /* comma after GPGGA */
    return collect_status_t::EMPTY;

  if ('\n' != buf[tip-1])
    return collect_status_t::EMPTY;

  if ('\r' != buf[tip-2])
    return collect_status_t::EMPTY;

  if ('*' != buf[tip-5])
    return collect_status_t::EMPTY;

  /* now calc checksum */
  for (size_t i=1; i<tip-5; i++)
    sum ^= buf[i];

  uint8_t sum_from_hex = (from_hex(buf[tip-4]) << 4) | from_hex(buf[tip-3]);
  if (sum != sum_from_hex)
    return collect_status_t::EMPTY;
  else
    return get_name((char *)&buf[1]);
}

/**
 *
 */
const char* NmeaParser::token(char *result, size_t N) {

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
void NmeaParser::unpack(nmea_gga_t &result) {
  char tmp[GPS_MAX_TOKEN_LEN];
  double c;

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
static float knots2mps(float knots) {
  return knots * 0.514444444;
}

/**
 *
 */
void NmeaParser::unpack(nmea_rmc_t &result) {
  char tmp[GPS_MAX_TOKEN_LEN];

  get_time(&result.time, &result.sec_round, token(tmp, 0));
  result.speed   = knots2mps(atof(token(tmp, 6)));
  result.course  = atof(token(tmp, 7));
  get_date(&result.time, token(tmp, 8));
}

/**
 *
 */
void NmeaParser::reset_collector(void) {
  tip = 0;
  maptip = 0;
  memset(this->buf, 0, sizeof(this->buf));
  memset(this->token_map, 0, sizeof(this->token_map));
  state = collect_state_t::START;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
NmeaParser::NmeaParser(void) :
tip(0),
maptip(0),
state(collect_state_t::START)
{
  memset(this->buf, 0, sizeof(this->buf));
  memset(this->token_map, 0, sizeof(this->token_map));
}

/**
 *
 */
collect_status_t NmeaParser::collect(uint8_t c) {
  collect_status_t ret = collect_status_t::EMPTY;

  /* prevent overflow */
  if ((tip >= GPS_MSG_LEN) || (maptip >= GPS_TOKEN_MAP_LEN)) {
    reset_collector();
  }

  switch (state) {
  case collect_state_t::START:
    if ('$' == c) {
      reset_collector();
      buf[0] = c;
      tip = 1;
      state = collect_state_t::DATA;
    }
    break;

  case collect_state_t::DATA:
    buf[tip] = c;
    if ((',' == c) || ('*' == c)) {
      token_map[maptip] = tip;
      maptip++;
    }
    tip++;
    if (tip >= sizeof(buf) - 4)
      reset_collector();
    if ('*' == c)
      state = collect_state_t::CHECKSUM1;
    break;

  case collect_state_t::CHECKSUM1:
    buf[tip] = c;
    tip++;
    state = collect_state_t::CHECKSUM2;
    break;

  case collect_state_t::CHECKSUM2:
    buf[tip] = c;
    tip++;
    state = collect_state_t::EOL_CR;
    break;

  case collect_state_t::EOL_CR:
    if ('\r' == c) {
      buf[tip] = c;
      tip++;
      state = collect_state_t::EOL_LF;
    }
    else
      reset_collector();
    break;

  case collect_state_t::EOL_LF:
    if ('\n' == c) {
      buf[tip] = c;
      tip++;
      ret = validate_sentece();
      if (collect_status_t::EMPTY == ret)
        reset_collector();
      state = collect_state_t::START;
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
