#ifndef ENGINE_HPP_
#define ENGINE_HPP_

#include "drivetrain/drivetrain_impact.hpp"
#include "drivetrain/drivetrain_pwm.hpp"

namespace Control {

class Engine {
public:
  Engine(PWM &pwm);
  void start(void);
  void stop(void);
  void update(const DrivetrainImpact &impact);
private:
  PWM &pwm;
  bool ready = false;
};

} /* namespace Control */

#endif /* ENGINE_HPP_ */
