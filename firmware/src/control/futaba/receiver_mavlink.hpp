#ifndef FUTABA_RECEIVER_MAVLINK_HPP_
#define FUTABA_RECEIVER_MAVLINK_HPP_

#include <futaba/receiver.hpp>

namespace control {

/**
 *
 */
class ReceiverMavlink : public Receiver {
public:
  void start(void);
  void stop(void);
  void update(RecevierOutput &result);
};

} /* namespace */

#endif /* FUTABA_RECEIVER_MAVLINK_HPP_ */
