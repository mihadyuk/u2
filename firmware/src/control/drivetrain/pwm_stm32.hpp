#ifndef PWM_STM32_HPP_
#define PWM_STM32_HPP_

#include "pwm_base.hpp"

//#define DRIVETRAIN_PWM_CLK        8000000 /* 1MHz clock */
//#define DRIVETRAIN_PWM_PERIOD     2000   /* 20000 == 50Hz pulse generation */

#define DRIVETRAIN_PWM_CLK        1000000 /* 1MHz clock */
#define DRIVETRAIN_PWM_PERIOD     20000   /* 20000 == 50Hz pulse generation */

namespace control {

/**
 *
 */
class PWMStm32 : public PWMBase {
public:
  PWMStm32(void);
  void start(void);
  void stop(void);
  void update(uint16_t pwm, size_t channel);
};

} /* namespace */

#endif /* PWM_STM32_HPP_ */
