#ifndef PWM_OVERRIDE_HPP_
#define PWM_OVERRIDE_HPP_

#define DRIVETRAIN_PWM_CHANNELS   8

namespace control {

/**
 * @brief   Output data from Futaba
 */
struct PwmVector {
  uint32_t mask = 0;
  uint16_t pwm[DRIVETRAIN_PWM_CHANNELS];    // pwm values in uS
};

} /* namespace */

#endif /* PWM_OVERRIDE_HPP_ */
