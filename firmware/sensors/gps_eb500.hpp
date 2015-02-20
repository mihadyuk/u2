#ifndef GPS_EB500_HPP_
#define GPS_EB500_HPP_

#define EVMSK_GPS_UPATED       (1UL << 0)

namespace gps {

/**
 *
 */
typedef struct {
  double    latitude;
  double    longitude;
  float     altitude;
  float     speed;
  float     course;
  struct tm time;
  bool      sec_round; /* no fractional part in seconds field*/
  bool      fix_valid;
} gps_data_t;

} /* namespace */

extern chibios_rt::EvtSource event_gps;

void GPSInit(void);
void GPSGetData(gps::gps_data_t &result);
void GPS_PPS_ISR_I(void);

#endif /* GPS_EB500_HPP_ */
