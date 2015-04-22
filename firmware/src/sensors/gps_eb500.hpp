#ifndef GPS_EB500_HPP_
#define GPS_EB500_HPP_

#define EVMSK_GPS_UPATED       (1UL << 0)

namespace gps {

/**
 *
 */
typedef struct {
  double    latitude;   // deg
  double    longitude;  // deg
  float     altitude;   // m
  float     speed;      // m/s
  float     course;     // deg
  struct tm time;
  bool      sec_round;  /* there is no fractional part in seconds' field */
  bool      fix_valid;  /* sets when fresh valid fix acquired, clears after structure reading */
} gps_data_t;

} /* namespace */

extern chibios_rt::EvtSource event_gps;

void GPSInit(void);
void GPSGetData(gps::gps_data_t &result);
void GPS_PPS_ISR_I(void);

#endif /* GPS_EB500_HPP_ */
