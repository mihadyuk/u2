#ifndef DRIVETRAIN_PWM_HPP_
#define DRIVETRAIN_PWM_HPP_

#define DRIVETRAIN_PWM_CHANNELS   4

#define DRIVETRAIN_PWM_CLK        1000000 /* 1MHz clock */
#define DRIVETRAIN_PWM_PERIOD     20000   /* 20000 == 50Hz pulse generation */

namespace control {

/**
 *
 */
typedef enum {
  PWM_CH_AIL = 0,
  PWM_CH_ELE = 1,
  PWM_CH_RUD = 2,
  PWM_CH_THR = 3,
  PWM_CH_THR_REVERSE = 0,
} pwm_ch_t;

/**
 *
 */
class PWM {
public:
  PWM(void);
  void start(void);
  void stop(void);
  void update(uint16_t pwm, size_t channel);
private:
  bool ready = false;
};

} /* namespace */

#endif /* DRIVETRAIN_PWM_HPP_ */
