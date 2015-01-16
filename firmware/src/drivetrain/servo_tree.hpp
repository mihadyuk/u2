#ifndef SERVO_TREE_HPP_
#define SERVO_TREE_HPP_

#include "drivetrain_impact.hpp"
#include "drivetrain_pwm.hpp"

namespace Drive {

class ServoTree {
public:
  ServoTree(PWM &pwm);
  void start(void);
  void stop(void);
  void update(const DrivetrainImpact &impact);
private:
  PWM &pwm;
  bool ready = false;
};

} /* namespace Drive */

#endif /* SERVO_TREE_HPP_ */
