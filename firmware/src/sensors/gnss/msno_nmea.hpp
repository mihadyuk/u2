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
  void inject(void) override;
  void subscribe_inject(void) override;
  void release_inject(void) override;

  mavlink_message_t *recv_msg = nullptr;
  chibios_rt::Mailbox<mavlink_message_t*, 1> inject_mailbox;
  SubscribeLink inject_link;
};

} // namespace

#endif /* MSNO_NMEA_HPP_ */
