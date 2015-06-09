#ifndef GNSS_RECEIVER_HPP_
#define GNSS_RECEIVER_HPP_

#include "acs_input.hpp"
#include "gnss_data.hpp"

#define EVMSK_GNSS_FRESH_VALID    (1UL << 0)

extern chibios_rt::EvtSource event_gps;

void GNSSInit(void);
void GNSSGet(gnss::gnss_data_t &result);
void GNSSSetSniffHook(SerialDriver *sdp);
void GNSSDeleteSniffHook(void);
void GNSS_PPS_ISR_I(void);

#endif /* GNSS_RECEIVER_HPP_ */
