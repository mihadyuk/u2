#ifndef MSNO_NMEA_HPP_
#define MSNO_NMEA_HPP_

#include "mav_postman.hpp"
#include "generic_nmea.hpp"

namespace gnss {

/**
 *
 */
class msnonmea : public GenericNMEA {
public:
  msnonmea(SerialDriver *sdp, uint32_t start_baudrate,
                              uint32_t working_baudrate);
private:
  void assist(void);
  void subscribe_assistance(void);
  void release_assistance(void);

  mavlink_message_t *recv_msg = nullptr;
  chibios_rt::Mailbox<mavlink_message_t*, 1> rtcm_mailbox;
  SubscribeLink gnss_assistance_link;
};

} // namespace

#endif /* MSNO_NMEA_HPP_ */
