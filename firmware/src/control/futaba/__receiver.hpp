#ifndef FUTABA_RECEIVER_HPP_
#define FUTABA_RECEIVER_HPP_

#include "pwm_vector.hpp"

#define FUTABA_RECEIVER_PWM_CHANNELS    8

namespace control {

class Receiver {
public:
  Receiver(systime_t timeout) : timeout(timeout) {
    ;
  }
  virtual msg_t update(uint16_t *pwm) const = 0;
  virtual void start(void) = 0;
  virtual void stop(void) = 0;

protected:
  bool ready = false;
  const systime_t timeout;
};

} /* namespace */

#endif /* FUTABA_RECEIVER_HPP_ */
