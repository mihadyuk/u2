#ifndef FUTABA_DATA_HPP_
#define FUTABA_DATA_HPP_

#include "control/impact.hpp"
#include "control/pwm_vector.hpp"

namespace control {

/**
 *
 */
typedef enum {
  OVERRIDE_LEVEL_NONE,
  OVERRIDE_LEVEL_NAVIGATOR,
  OVERRIDE_LEVEL_STABILIZER,
  OVERRIDE_LEVEL_PWM,
} override_level_t;

/**
 * @brief   Output data from ACS
 */
typedef struct {
  override_level_t override_level;
  Impact impact;
  PwmVector pwm_vector;
}FutabaData;

} /* namespace */

#endif /* FUTABA_DATA_HPP_ */
