#ifndef FUTABA_DATA_HPP_
#define FUTABA_DATA_HPP_

#include "target_direction.hpp"
#include "impact.hpp"
#include "pwm_vector.hpp"
#include "target_attitude.hpp"

namespace control {

/**
 *
 */
enum class OverrideLevel {
  pwm,
  impact,
  attitude,
  direction,
  none,
  enum_end
};

/**
 * @brief   Output data from Futaba
 */
typedef struct {
  OverrideLevel level;
  TargetDirection direction;
  TargetAttitude attitude;
  Impact impact;
  PwmVector pwm_vector;
} FutabaData;

} /* namespace */

#endif /* FUTABA_DATA_HPP_ */
