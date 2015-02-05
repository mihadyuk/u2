#ifndef NMEA_HPP_
#define NMEA_HPP_

/* From standard: A sentence may contain up to 80 characters plus "$" and CR/LF */
#define GPS_MSG_LEN     84

namespace gps {

/**
 *
 */
typedef struct {
  double        latitude;
  double        longitude;
  double        altitude;
  unsigned long time;
  int           satellites;
  int           quality;
  double        hdop;
  double        geoid;
} nmeap_gga_t;

/**
 *
 */
typedef struct {
  unsigned long time;
  char          warn;
  double        latitude;
  double        longitude;
  double        speed;
  double        course;
  unsigned long date;
  double        magvar;
} nmeap_rmc_t;

/**
 *
 */
enum class collect_status_t {
  EMPTY,
  GPGGA,
  GPRMC,
  UNKNOWN
};

/**
 *
 */
enum class collect_state_t {
  START,      /* wait '$' sign */
  DATA,       /* collect data until '*' sign */
  CHECKSUM1,  /* collect 1st byte of checksum */
  CHECKSUM2,  /* collect 2nd byte of checksum */
  EOL_CR,     /* wait CR symbol */
  EOL_LF      /* wait LF symbol */
};

/**
 *
 */
class NmeaParser {
public:
  NmeaParser(void);
  collect_status_t collect(uint8_t byte);
  void unpack(nmeap_rmc_t *result);
  void unpack(nmeap_gga_t *result);
private:
  void reset_collector(void);
  collect_status_t verify_sentece(void);
  collect_status_t get_name(const char *name);
  size_t tip;
  collect_state_t state;
  uint8_t buf[GPS_MSG_LEN];
};

} /* namespace */

gps::collect_status_t nmea_test_gga(void);
gps::collect_status_t nmea_test_rmc(void);

#endif /* NMEA_HPP_ */
