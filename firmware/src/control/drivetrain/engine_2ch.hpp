#ifndef ENGINE_2CH_HPP_
#define ENGINE_2CH_HPP_

#include "engine.hpp"
#include "drivetrain_impact.hpp"
#include "drivetrain/drivetrain_pwm.hpp"

namespace control {

class Engine2ch : public Engine {
public:
  Engine2ch(PWM &pwm);
private:
  void start_impl(void);
  void update_impl(const DrivetrainImpact &impact);
  int32_t float2pwm(float thr);
  void write_thrust_pwm(int32_t thrpwm);
  PWM &pwm;
  const uint32_t *thr_dz  = nullptr;
};

} /* namespace */

#endif /* ENGINE_2CH_HPP_ */
