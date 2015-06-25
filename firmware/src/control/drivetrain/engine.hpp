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
  Engine(void);
  void start(void);
  void stop(void);
  void arm(void);
  void disarm(void);
  void update(const DrivetrainImpact &impact);
private:
  virtual void start_impl(void) = 0;
  virtual void update_impl(const DrivetrainImpact &impact) = 0;
protected:
  void thrust2mavlink(float thr);
  EngineState state;
};

} /* namespace */

#endif /* ENGINE_HPP_ */
