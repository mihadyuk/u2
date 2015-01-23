#ifndef DRIVETRAIN_PWM_OVERRIDE_HPP_
#define DRIVETRAIN_PWM_OVERRIDE_HPP_

#include "array_len.hpp"
#include <bitset>

namespace Control {

/**
 * @brief   Output data from Futaba
 */
struct PwmOverride {
  PwmOverride(void) {
    for (size_t i=0; i<ArrayLen(a); i++)
      a[i] = 1500;
  };
  std::bitset<DRIVETRAIN_PWM_CHANNELS> mask;
  uint16_t a[DRIVETRAIN_PWM_CHANNELS];    // pwm values in uS
};

} /* namespace Control */

#endif /* DRIVETRAIN_PWM_OVERRIDE_HPP_ */
