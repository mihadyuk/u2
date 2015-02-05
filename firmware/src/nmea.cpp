#include <time.h>
#include <cmath>
#include <cstring>

#include "main.h"
#include "nmea.hpp"

using namespace gps;

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
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

static int32_t parse_decimal(uint8_t *p);
static int32_t parse_degrees(uint8_t *p);
static uint32_t gpsatol(const uint8_t *str);
static bool gpsisdigit(char c);
static uint8_t from_hex(uint8_t a);

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
collect_status_t NmeaParser::verify_sentece(void) {
  uint8_t sum = 0;
  uint8_t sum_from_hex;

  for (size_t i=1; i<tip-5; i++)
    sum ^= buf[i];

  sum_from_hex = (from_hex(buf[tip-4]) << 4) | from_hex(buf[tip-3]);
  if (sum != sum_from_hex)
    return collect_status_t::EMPTY;
  else
    return get_name((char *)&buf[1]);
}

/**
 *
 */
void NmeaParser::reset_collector(void) {
  tip = 0;
  memset(this->buf, 0, sizeof(buf));
  state = collect_state_t::START;
}



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
static void get_time(struct tm *timp, uint8_t *buft, uint8_t *bufd){
  timp->tm_hour = 10 * (buft[0] - '0') + (buft[1] - '0');
  timp->tm_min  = 10 * (buft[2] - '0') + (buft[3] - '0');
  timp->tm_sec  = 10 * (buft[4] - '0') + (buft[5] - '0');

  timp->tm_mday = 10 * (bufd[0] - '0') + (bufd[1] - '0');
  timp->tm_mon  = 10 * (bufd[2] - '0') + (bufd[3] - '0') - 1;
  timp->tm_year = 10 * (bufd[4] - '0') + (bufd[5] - '0') + 2000 - 1900;
}


/**
 * Возвращает значение с фиксированной точкой с точностью 2 знака после запятой
 */
static int32_t parse_decimal(uint8_t *p){
  bool isneg = (*p == '-');   /* обработаем наличие знака "-" */
  if (isneg) ++p;
  uint32_t ret = gpsatol(p);  /* сделаем заготовку для возвращаемого значения */
  ret = ret * 100UL;          /* сделаем место для 2 знаков после запятой */

  while (gpsisdigit(*p)) ++p; /* пропустим все знаки до запятой - мы их уже обработали */
  if (*p == '.'){             /* запишем 2 знака после запятой */
    if (gpsisdigit(p[1])){
      ret += 10 * (p[1] - '0');
      if (gpsisdigit(p[2]))
        ret += p[2] - '0';
    }
  }
  return isneg ? -ret : ret;
}

static int32_t parse_degrees(uint8_t *p){
  uint32_t left = gpsatol(p);                       /* читаем первую часть (ddmm) */
  uint32_t tenk_minutes = (left % 100UL) * 10000UL; /* отделяем целые части минут */

  while (gpsisdigit(*p)) ++p;
  if (*p == '.'){
    uint32_t mult = 1000; /* только 3 знака после запятой */
    while (gpsisdigit(*++p)){
      tenk_minutes += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left / 100) * 100000 + tenk_minutes / 6;
}

static uint32_t gpsatol(const uint8_t *str){
  uint32_t ret = 0;
  while (gpsisdigit(*str))
    ret = 10 * ret + *str++ - '0';
  return ret;
}

static bool gpsisdigit(char c){
  return c >= '0' && c <= '9';
}

/**
 *
 */
void NmeaParser::unpack(nmeap_gga_t *result) {

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
    state(collect_state_t::START),
    tip(0)
{
  memset(this->buf, 0, sizeof(buf));
}

/**
 *
 */
collect_status_t NmeaParser::collect(uint8_t byte) {
  collect_status_t ret = collect_status_t::EMPTY;

  switch (state) {
  case collect_state_t::START:
    if ('$' == byte) {
      buf[0] = byte;
      tip = 1;
      state = collect_state_t::DATA;
    }
    else
      reset_collector();
    break;

  case collect_state_t::DATA:
    buf[tip] = byte;
    tip++;
    if (tip >= sizeof(buf) - 4)
      reset_collector();
    if ('*' == byte)
      state = collect_state_t::CHECKSUM1;
    break;

  case collect_state_t::CHECKSUM1:
    buf[tip] = byte;
    tip++;
    state = collect_state_t::CHECKSUM2;
    break;

  case collect_state_t::CHECKSUM2:
    buf[tip] = byte;
    tip++;
    state = collect_state_t::EOL_CR;
    break;

  case collect_state_t::EOL_CR:
    if ('\r' == byte) {
      buf[tip] = byte;
      tip++;
      state = collect_state_t::EOL_LF;
    }
    else
      reset_collector();
    break;

  case collect_state_t::EOL_LF:
    if ('\n' == byte) {
      buf[tip] = byte;
      tip++;
      ret = verify_sentece();
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



/**
 *
 */
static const uint8_t gga1[] = "$GPGGA,115436.000,5354.713670,N,02725.690517,E,1,5,2.01,266.711,M,26.294,M,,*5D\r\n";
static const uint8_t gga2[] = "$GPGGA,000103.037,,,,,0,0,,,M,,M,,*4E";

static nmeap_gga_t unpacked_gga;

/**
 *
 */
collect_status_t nmea_test_gga(void) {
  collect_status_t ret;
  NmeaParser myparser;

  for (size_t i=0; i<sizeof(gga1); i++) {
    ret = myparser.collect(gga1[i]);
    if (ret != collect_status_t::GPGGA)
      myparser.unpack(&unpacked_gga);
  }

  return ret;
}






