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
protected:
  virtual void start_impl(void) = 0;
  virtual void update_impl(const DrivetrainImpact &impact) = 0;
  void thrust2mavlink(float thr);
  PWM &pwm;
  EngineState state;
};

} /* namespace */

#endif /* ENGINE_HPP_ */
