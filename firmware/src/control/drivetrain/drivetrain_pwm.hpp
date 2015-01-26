#ifndef DRIVETRAIN_PWM_HPP_
#define DRIVETRAIN_PWM_HPP_

#define DRIVETRAIN_PWM_CLK        1000000 /* 1MHz clock */
#define DRIVETRAIN_PWM_PERIOD     20000   /* 20000 == 50Hz pulse generation */

#include <pwm_vector.hpp>

namespace control {

/**
 *
 */
typedef enum {
  PWM_CH_AIL = 0,
  PWM_CH_ELE = 1,
  PWM_CH_RUD = 2,
  PWM_CH_RESERVED_0 = 3,
  PWM_CH_THRUST_FORTH = 4,
  PWM_CH_THRUST_BACK = 5,
  PWM_CH_RESERVED_2 = 6,
  PWM_CH_RESERVED_3 = 7,
} pwm_ch_t;

class PWM {
public:
  PWM(void);
  void start(void);
  void stop(void);
  void update(uint16_t pwm, size_t channel);
  void futaba_override(const PwmVector &override);
private:
  bool ready = false;
};

} /* namespace */

#endif /* DRIVETRAIN_PWM_HPP_ */
