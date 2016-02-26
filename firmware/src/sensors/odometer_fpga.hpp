#ifndef ODOMETER_FPGA_HPP_
#define ODOMETER_FPGA_HPP_

#include "odometer.hpp"
#include "alpha_beta.hpp"
#include "median.hpp"
#include "fpga_pwm.h"

/**
 *
 */
class OdometerFPGA : public Odometer {
public:
  OdometerFPGA(const FpgaPwm *fpgaicup);
private:
  void start_impl(void);
  void stop_impl(void);
  void update_impl(odometer_data_t &result, float dT);
  filters::AlphaBeta<float, 4> filter_alphabeta;
  filters::Median<float, 3> filter_median;
  const FpgaPwm *fpgaicup;
};

#endif /* ODOMETER_FPGA_HPP_ */
