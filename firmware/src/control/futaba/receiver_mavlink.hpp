#ifndef FUTABA_RECEIVER_MAVLINK_HPP_
#define FUTABA_RECEIVER_MAVLINK_HPP_

#include "mav_postman.hpp"
#include "futaba/receiver.hpp"

namespace control {

/**
 *
 */
class ReceiverMavlink : public Receiver {
public:
  ReceiverMavlink(void);
  void start(void);
  void stop(void);
  void update(RecevierOutput &result);
private:
  THD_WORKING_AREA(RCMavlinkThreadWA, 256);
  static THD_FUNCTION(RCMavlinkThread, arg);
  thread_t *worker = nullptr;

  mavlink_message_t *recv_msg = nullptr;
  chibios_rt::Mailbox<mavlink_message_t*, 1> control_mailbox;
  SubscribeLink control_link;
};

} /* namespace */

#endif /* FUTABA_RECEIVER_MAVLINK_HPP_ */
