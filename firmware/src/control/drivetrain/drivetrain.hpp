#ifndef DRIVETRAIN_HPP_
#define DRIVETRAIN_HPP_

#include "drivetrain_impact.hpp"
#include "pwm_fpga.hpp"
#include "pwm_stm32.hpp"
#include "servo_tree.hpp"
#include "engine_1ch.hpp"
#include "engine_2ch.hpp"

namespace control {

/**
 * @brief   Convert impact values to servo and engine values.
 */
class Drivetrain {
public:
  Drivetrain(void);
  void start(void);
  void stop(void);
  void arm(void);
  void disarm(void);
  msg_t update(const DrivetrainImpact &impact);
  uint32_t capabilities(void);
private:
  bool ready = false;

#if defined(BOARD_BEZVODIATEL)
  PWMStm32 pwm;
#elif defined(BOARD_MNU)
  PWMFPGA pwm;
#else
#error "unknown board"
#endif
  Engine *engine;
  ServoTree servo;
};

} /* namespace */

#endif /* DRIVETRAIN_HPP_ */
