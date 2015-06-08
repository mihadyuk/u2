#ifndef EB500_HPP_
#define EB500_HPP_

#include "acs_input.hpp"
#include "gps_data.hpp"

#define EVMSK_GPS_FRESH_VALID   (1UL << 0)

extern chibios_rt::EvtSource event_gps;

void GPSInit(void);
void GPSGet(gps_data_t &result);
void GPSSetDumpHook(SerialDriver *sdp);
void GPSDeleteDumpHook(void);
void GPS_PPS_ISR_I(void);

#endif /* EB500_HPP_ */
