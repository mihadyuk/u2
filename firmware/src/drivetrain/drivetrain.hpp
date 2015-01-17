#ifndef DRIVETRAIN_HPP_
#define DRIVETRAIN_HPP_

#include "drivetrain_impact.hpp"
#include "drivetrain_pwm.hpp"
#include "servo_tree.hpp"
#include "engine.hpp"

namespace Drive {

/**
 * @brief   Convert different impact values to servo angles
 */
class Drivetrain {
public:
  Drivetrain(const DrivetrainImpact &impact);
  void start(void);
  void stop(void);
  msg_t update();

private:
  bool ready = false;
  const DrivetrainImpact &impact;
  PWM pwm;
  Engine engine;
  ServoTree servo;
};

} /* namespace Drive */

#endif /* DRIVETRAIN_HPP_ */
