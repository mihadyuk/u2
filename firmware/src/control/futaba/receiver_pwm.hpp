#ifndef RECEIVER_PWM_HPP_
#define RECEIVER_PWM_HPP_

#include "receiver.hpp"

namespace control {

/**
 *
 */
class ReceiverPWM : public Receiver {
public:
  void start(systime_t timeout);
  void stop(void);
  void update(receiver_data_t &result) const;
  friend void futaba_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p);
};

} /* namespace */

#endif /* RECEIVER_PWM_HPP_ */
