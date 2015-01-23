#ifndef PWM_RECEIVER_PLANE_HPP_
#define PWM_RECEIVER_PLANE_HPP_

#include <control/futaba/pwm_receiver.hpp>

#include "pwm2rs.hpp"

#define PWM_AIL_CHANNEL             0
#define PWM_ELE_CHANNEL             1
#define PWM_RUD_CHANNEL             2
#define PWM_THRUST_CHANNEL          3
#define PWM_FLAP_CHANNEL            4
#define PWM_STRUT_CHANNEL           5
#define PWM_BREAK_CHANNEL           6
#define PWM_CHUTE_CHANNEL           7

//#define PWM_RESERVED_CHANNEL_8      8
//#define PWM_SWITCH_MANUAL_CHANNEL   9
#define PWM_SWITCH_MANUAL_CHANNEL   7
#define PWM_RESERVED_CHANNEL_9      9

#define PWM_RESERVED_CHANNEL_10     10
#define PWM_RESERVED_CHANNEL_11     11
#define PWM_RESERVED_CHANNEL_12     12
#define PWM_RESERVED_CHANNEL_13     13

/**
 *
 */
class PWMReceiverPlane : public PWMReceiver{
public:
  PWMReceiverPlane(void);
  void start(void);
  void stop(void);

private:
  bool update_impl(PwmVector *pwm, systime_t timeout);
  PWM2RS pwm2rs;
};



#endif /* PWM_RECEIVER_PLANE_HPP_ */
