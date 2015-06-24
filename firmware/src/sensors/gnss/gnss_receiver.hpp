#ifndef GNSS_RECEIVER_HPP_
#define GNSS_RECEIVER_HPP_

#include "acs_input.hpp"
#include "gnss_data.hpp"
#include "mavlink_local.hpp"
#include "mav_mail.hpp"

#define EVMSK_GNSS_FRESH_VALID    (1UL << 0)

extern chibios_rt::EvtSource event_gnss;

#define GNSS_MAX_SUBSCRIBERS      4

#define GNSS_MAVLINK_HDG_UNKNOWN  65535

namespace gnss {

/**
 *
 */
class GNSSReceiver {
public:
  GNSSReceiver(SerialDriver *sdp, uint32_t start_baudrate,
                                  uint32_t working_baudrate);
  virtual void start(void) = 0;
  void stop(void);
  void getCache(gnss::gnss_data_t &result);
  void subscribe(gnss::gnss_data_t* result);
  void unsubscribe(gnss::gnss_data_t* result);
  void setSniffer(SerialDriver *sdp);
  void deleteSniffer(void);
  static void GNSS_PPS_ISR_I(void);
protected:
  THD_WORKING_AREA(gnssRxThreadWA, 400);
  void log_append(const mavlink_gps_raw_int_t *msg);
  void acquire(void);
  void release(void);
  gnss_data_t* spamlist[GNSS_MAX_SUBSCRIBERS] = {};
  bool ready = false;
  thread_t *worker = nullptr;
  SerialDriver *sniff_sdp = nullptr;
  SerialDriver *sdp = nullptr;
  const uint32_t start_baudrate;
  const uint32_t working_baudrate;
  gnss_data_t cache;
  mavMail gps_raw_int_mail;
  static chibios_rt::BinarySemaphore pps_sem;
  static chibios_rt::BinarySemaphore protect_sem;
};

} // namespace

#endif /* GNSS_RECEIVER_HPP_ */
