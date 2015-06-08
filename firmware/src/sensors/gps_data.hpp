#ifndef GPS_DATA_HPP_
#define GPS_DATA_HPP_

/**
 *
 */
struct gps_data_t {
  gps_data_t(void) {
    memset(this, 0, sizeof(*this));
  }
  double    latitude;   // deg
  double    longitude;  // deg
  float     altitude;   // m
  float     speed;      // m/s
  float     course;     // deg
  struct tm time;
  bool      sec_round;  /* there is no fractional part in seconds' field */
};

#endif /* GPS_DATA_HPP_ */
