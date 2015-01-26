#ifndef SERVO_TREE_HPP_
#define SERVO_TREE_HPP_

#include "impact.hpp"
#include "drivetrain_pwm.hpp"

namespace control {

class ServoTree {
public:
  ServoTree(PWM &pwm);
  void start(void);
  void stop(void);
  void update(const Impact &impact);
private:
  PWM &pwm;
  bool ready = false;
  const uint32_t *rud_min = nullptr, *rud_mid = nullptr, *rud_max = nullptr;
};

} /* namespace */

#endif /* SERVO_TREE_HPP_ */
