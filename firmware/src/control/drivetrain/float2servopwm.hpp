#ifndef FLOAT2SERVOPWM_HPP_
#define FLOAT2SERVOPWM_HPP_

#include "putinrange.hpp"

namespace control {

#define SRV_MIN 1000
#define SRV_MID 1500
#define SRV_MAX 2000

/**
 *
 */
static inline uint16_t float2servo_pwm(float a, int min, int mid, int max) {
  uint16_t ret;

  a = putinrange(a, -1, 1);

  if (a > 0)
    ret = mid + (max - mid) * a;
  else
    ret = mid + (mid - min) * a;

  return putinrange(ret, SRV_MIN, SRV_MAX);
}

} // namespace

#endif /* FLOAT2SERVOPWM_HPP_ */
