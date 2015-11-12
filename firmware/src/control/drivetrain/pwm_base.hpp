#ifndef PWM_BASE_HPP_
#define PWM_BASE_HPP_

#define DRIVETRAIN_PWM_CHANNELS   4

namespace control {

/**
 *
 */
#if defined(BOARD_BEZVODIATEL)
typedef enum {
  PWM_CH_AIL = 0,
  PWM_CH_ELE = 1,
  PWM_CH_RUD = 2,
  PWM_CH_THR = 3,
  PWM_CH_THR_REVERSE = 0, // for simple PWM to brushed motor
} pwm_ch_t;
#elif defined(BOARD_MNU)
typedef enum {
  PWM_CH_AIL = 2,
  PWM_CH_ELE = 3,
  PWM_CH_RUD = 0,
  PWM_CH_THR = 1,
  PWM_CH_THR_REVERSE = 0, // for simple PWM to brushed motor
} pwm_ch_t;
#else
#error "board unsupported"
#endif

/**
 *
 */
class PWMBase {
public:
  PWMBase(void) {ready = false;}
  virtual void start(void) = 0;
  virtual void stop(void) = 0;
  virtual void update(uint16_t pwm, size_t channel) = 0;
protected:
  bool ready;
};

} /* namespace */

#endif /* PWM_BASE_HPP_ */
