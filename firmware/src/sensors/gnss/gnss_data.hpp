#ifndef GPS_DATA_HPP_
#define GPS_DATA_HPP_

namespace gnss {
/**
 *
 */
enum class speed_t {
  NONE = 0,     /* GNSS receiver does not provide speed data */
  SPEED_COURSE, /* 2 components from RMC NMEA message: speed over ground + course */
  VECTOR_3D     /* full 3-component vector */
};

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
  speed_t   speed_type;
  struct tm time;
  bool      sec_round;  /* there is no fractional part in seconds' field */
};
}
#endif /* GPS_DATA_HPP_ */
