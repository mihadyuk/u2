#ifndef DRIVETRAIN_HPP_
#define DRIVETRAIN_HPP_

#include "drivetrain_impact.hpp"
#include "servo_tree.hpp"
#include "engine.hpp"

namespace control {

/**
 * @brief   Convert impact values to servo and engine values.
 */
class Drivetrain {
public:
  Drivetrain(void);
  void start(void);
  void stop(void);
  msg_t update(const DrivetrainImpact &impact);
  uint32_t capabilities(void);

private:
  bool ready = false;
  PWM pwm;
  Engine engine;
  ServoTree servo;
};

} /* namespace */

#endif /* DRIVETRAIN_HPP_ */
