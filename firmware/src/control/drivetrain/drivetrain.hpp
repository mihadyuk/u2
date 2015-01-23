#ifndef DRIVETRAIN_HPP_
#define DRIVETRAIN_HPP_

#include "servo_tree.hpp"
#include "drivetrain_impact.hpp"
#include "drivetrain_pwm.hpp"
#include "engine.hpp"

namespace Control {

/**
 * @brief   Convert different impact values to servo angles
 */
class Drivetrain {
public:
  Drivetrain(void);
  void start(void);
  void stop(void);
  msg_t update(const DrivetrainImpact &impact);
  void futaba_override(const PwmOverride &override);

private:
  bool ready = false;
  PWM pwm;
  Engine engine;
  ServoTree servo;
};

} /* namespace Control */

#endif /* DRIVETRAIN_HPP_ */
