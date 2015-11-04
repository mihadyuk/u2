#ifndef PWM_FPGA_HPP_
#define PWM_FPGA_HPP_

#include "pwm_base.hpp"

namespace control {

/**
 *
 */
class PWMFPGA : public PWMBase {
public:
  PWMFPGA(void);
  void start(void);
  void stop(void);
  void update(uint16_t pwm, size_t channel);
};

} /* namespace */

#endif /* PWM_FPGA_HPP_ */
