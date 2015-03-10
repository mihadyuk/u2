#ifndef SERVO_TREE_HPP_
#define SERVO_TREE_HPP_

#include "drivetrain_impact.hpp"
#include "drivetrain_pwm.hpp"

namespace control {

class ServoTree {
public:
  ServoTree(PWM &pwm);
  void start(void);
  void stop(void);
  void update(const DrivetrainImpact &impact);
private:
  PWM &pwm;
  bool ready = false;
  const uint32_t *ail_min = nullptr, *ail_mid = nullptr, *ail_max = nullptr;
  const uint32_t *ele_min = nullptr, *ele_mid = nullptr, *ele_max = nullptr;
  const uint32_t *rud_min = nullptr, *rud_mid = nullptr, *rud_max = nullptr;
};

} /* namespace */

#endif /* SERVO_TREE_HPP_ */
