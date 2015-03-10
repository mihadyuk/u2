#ifndef RECEIVER_PPM_HPP_
#define RECEIVER_PPM_HPP_

#include <futaba/receiver.hpp>

namespace control {

/**
 *
 */
class ReceiverPPM : public Receiver {
public:
  void start(systime_t timeout);
  void stop(void);
  void update(receiver_data_t &result);
};

} /* namespace */

#endif /* RECEIVER_PPM_HPP_ */
