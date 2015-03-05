#ifndef FUTABA_RECEIVER_MAVLINK_HPP_
#define FUTABA_RECEIVER_MAVLINK_HPP_

#include <futaba/receiver.hpp>

namespace control {

/**
 *
 */
class ReceiverMavlink : public Receiver {
public:
  void start(systime_t timeout);
  void stop(void);
  void update(receiver_data_t &result) const;
};

} /* namespace */

#endif /* FUTABA_RECEIVER_MAVLINK_HPP_ */
