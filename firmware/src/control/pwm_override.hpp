#ifndef PWM_OVERRIDE_HPP_
#define PWM_OVERRIDE_HPP_

#include "array_len.hpp"
#include <bitset>

namespace control {

/**
 * @brief   Output data from Futaba
 */
struct PwmOverride {
  uint32_t mask = 0;
  uint16_t a[DRIVETRAIN_PWM_CHANNELS];    // pwm values in uS
};

} /* namespace */

#endif /* PWM_OVERRIDE_HPP_ */
