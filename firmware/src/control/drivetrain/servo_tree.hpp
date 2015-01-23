#ifndef SERVO_TREE_HPP_
#define SERVO_TREE_HPP_

#include "drivetrain_impact.hpp"
#include "drivetrain_pwm.hpp"

namespace Control {

class ServoTree {
public:
  ServoTree(PWM &pwm);
  void start(void);
  void stop(void);
  void update(const DrivetrainImpact &impact);
private:
  PWM &pwm;
  bool ready = false;
  const uint32_t *rud_min, *rud_mid, *rud_max;
};

} /* namespace Control */

#endif /* SERVO_TREE_HPP_ */
