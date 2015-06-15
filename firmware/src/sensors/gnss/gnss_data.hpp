#ifndef GPS_DATA_HPP_
#define GPS_DATA_HPP_

namespace gnss {

/**
 *
 */
#define GNSS_SPEED_TYPE_NONE            0
#define GNSS_SPEED_TYPE_SPEED_COURSE    1
#define GNSS_SPEED_TYPE_VECTOR_3D       2

/**
 *
 */
struct gnss_data_t {
  gnss_data_t(void) {
    memset(this, 0, sizeof(*this));
  }
  double    latitude;   // deg
  double    longitude;  // deg
  float     altitude;   // m
  float     speed;      // m/s
  float     course;     // deg
  float     v[3];       // 3 components of speed (m/s, NED)
  bool      fresh;      // sync primitive. Receiver can only set it, consumer can only clear it
  struct tm time;
  uint16_t  msec;       // fractional part of seconds
  uint8_t   fix;        // GNSS fix type
  uint8_t   speed_type;
};

} // namespace

#endif /* GPS_DATA_HPP_ */
