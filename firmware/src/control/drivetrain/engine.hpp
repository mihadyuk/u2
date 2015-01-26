#ifndef ENGINE_HPP_
#define ENGINE_HPP_

#include "impact.hpp"
#include "drivetrain/drivetrain_pwm.hpp"

namespace control {

class Engine {
public:
  Engine(PWM &pwm);
  void start(void);
  void stop(void);
  void update(const Impact &impact);
private:
  PWM &pwm;
  bool ready = false;
};

} /* namespace */

#endif /* ENGINE_HPP_ */
