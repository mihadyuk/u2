#ifndef GPS_DATA_HPP_
#define GPS_DATA_HPP_

namespace gnss {

/**
 *
 */
enum class speed_t {
  SPEED_COURSE, /* 2 components from RMC NMEA message: speed over ground + course */
  VECTOR_3D,    /* full 3-component vector */
  BOTH
};

/**
 *
 */
struct gnss_data_t {
  gnss_data_t(void) {
    memset(this, 0, sizeof(*this));
    hdop = 1;
    vdop = 1;
  }
  double    latitude;   // rad
  double    longitude;  // rad
  float     altitude;   // m
  float     speed;      // m/s
  float     course;     // rad
  float     v[3];       // 3 components of speed (m/s, NED)
#warning "add dops"
  float     hdop;
  float     vdop;
  bool      fresh;      // sync primitive. Receiver can only set it, consumer can only clear it
  speed_t   speed_type;
  struct tm time;
  uint16_t  msec;       // Fractional part of time
  uint8_t   fix;        // GNSS fix type
};

} // namespace

#endif /* GPS_DATA_HPP_ */
