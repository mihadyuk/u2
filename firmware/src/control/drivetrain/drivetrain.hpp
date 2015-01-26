#ifndef DRIVETRAIN_HPP_
#define DRIVETRAIN_HPP_

#include "servo_tree.hpp"
#include "impact.hpp"
#include "drivetrain_pwm.hpp"
#include "engine.hpp"

namespace control {

/**
 * @brief   Convert different impact values to servo angles
 */
class Drivetrain {
public:
  Drivetrain(void);
  void start(void);
  void stop(void);
  msg_t update(const Impact &impact);
  void futaba_override(const PwmOverride &override);

private:
  bool ready = false;
  PWM pwm;
  Engine engine;
  ServoTree servo;
};

} /* namespace */

#endif /* DRIVETRAIN_HPP_ */
