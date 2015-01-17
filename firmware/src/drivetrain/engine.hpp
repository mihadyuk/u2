#ifndef ENGINE_HPP_
#define ENGINE_HPP_

#include "drivetrain_impact.hpp"
#include "drivetrain_pwm.hpp"

namespace Drive {

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

} /* namespace Drive */

#endif /* ENGINE_HPP_ */
