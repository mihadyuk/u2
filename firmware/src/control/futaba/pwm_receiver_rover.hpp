#ifndef PWM_RECEIVER_ROVER_HPP_
#define PWM_RECEIVER_ROVER_HPP_

#include <control/futaba/pwm_receiver.hpp>

/**
 *
 */
class PWMReceiverRover : public PWMReceiver{
public:
  PWMReceiverRover(void);
  void start(void);
  void stop(void);

private:
  bool update_impl(PwmVector *pwm, systime_t timeout);
};



#endif /* PWM_RECEIVER_ROVER_HPP_ */
