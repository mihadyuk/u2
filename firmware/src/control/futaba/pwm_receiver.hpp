#ifndef PWM_RECEIVER_HPP_
#define PWM_RECEIVER_HPP_

#include "pwm_vector.hpp"

class PWMReceiver {
public:
  bool update(PwmVector *pwm, systime_t timeout);
  virtual void start(void) = 0;
  virtual void stop(void) = 0;

protected:
  bool ready;

private:
  virtual bool update_impl(PwmVector *pwm, systime_t timeout) = 0;
};

#endif /* PWM_RECEIVER_HPP_ */
