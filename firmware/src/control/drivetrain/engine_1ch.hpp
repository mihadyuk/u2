#ifndef ENGINE_1CH_HPP_
#define ENGINE_1CH_HPP_

#include "engine.hpp"
#include "drivetrain_impact.hpp"
#include "drivetrain/drivetrain_pwm.hpp"

namespace control {

/**
 *
 */
class Engine1ch : public Engine {
public:
  Engine1ch(PWM &pwm);
private:
  void start_impl(void);
  void update_impl(const DrivetrainImpact &impact);
  const uint32_t *thr_min = nullptr;
  const uint32_t *thr_mid = nullptr;
  const uint32_t *thr_max = nullptr;
};

} /* namespace */

#endif /* ENGINE_1CH_HPP_ */
