#ifndef ENGINE_HPP_
#define ENGINE_HPP_

#include "drivetrain_impact.hpp"
#include "drivetrain/drivetrain_pwm.hpp"

namespace control {

/**
 *
 */
enum class EngineState {
  uninit,
  disarmed,
  armed
};

/**
 *
 */
class Engine {
public:
  Engine(PWM &pwm);
  void start(void);
  void stop(void);
  void arm(void);
  void disarm(void);
  void update(const DrivetrainImpact &impact);
private:
  void thrust2mavlink(float thr);
  PWM &pwm;
  EngineState state;
  const uint32_t *thr_min = nullptr;
  const uint32_t *thr_mid = nullptr;
  const uint32_t *thr_max = nullptr;
};

} /* namespace */

#endif /* ENGINE_HPP_ */
