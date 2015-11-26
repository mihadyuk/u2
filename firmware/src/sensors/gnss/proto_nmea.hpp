#ifndef PROTO_NMEA_HPP_
#define PROTO_NMEA_HPP_

#include "string.h" // for memset

/* From standard: A sentence may contain up to 80 printable characters
 * plus CR/LF plus 2 padding bytes for memory alignment */
#define GPS_MSG_LEN           84
#define GPS_TOKEN_MAP_LEN     24
#define GPS_MAX_TOKEN_LEN     16

namespace gnss {

/**
 *
 */
struct nmea_gga_t {
  nmea_gga_t(void) {
    memset(this, 0, sizeof(*this));
  }
  double    latitude;     // deg
  double    longitude;    // deg
  float     altitude;     // m
  float     hdop;
  float     geoid;
  struct tm time;
  uint16_t  msec;         // milliseconds from message timestamp
  uint8_t   satellites;
  uint8_t   fix;
};

/**
 *
 */
struct nmea_rmc_t {
  nmea_rmc_t(void) {
    memset(this, 0, sizeof(*this));
  }
  float     speed;    // m/s
  float     course;   // deg
  struct tm time;
  uint16_t  msec;     // milliseconds from message timestamp
};

/**
 *
 */
enum class sentence_type_t {
  EMPTY,
  GGA,
  RMC,
  UNKNOWN
};

/**
 *
 */
enum class nmea_collect_state_t {
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
class ProtoNmea {
public:
  ProtoNmea(void);
  sentence_type_t collect(uint8_t byte);
  void unpack(nmea_rmc_t &result);
  void unpack(nmea_gga_t &result);
  uint8_t checksum(const uint8_t *data, size_t len);
  void checksum2str(uint8_t sum, char *str);
  uint8_t checksumFromStr(const char *str);
  void seal(char *msg);
private:
  bool checksum_autotest(void);
  bool _autotest(const char *sentence);
  void reset_collector(void);
  const char* token(char *result, size_t number);
  sentence_type_t validate_sentence(void);
  sentence_type_t get_name(const char *name);
  size_t tip;
  size_t maptip;
  nmea_collect_state_t state;
  uint8_t buf[GPS_MSG_LEN];
  uint8_t token_map[GPS_TOKEN_MAP_LEN];
};

} /* namespace */

#endif /* PROTO_NMEA_HPP_ */
