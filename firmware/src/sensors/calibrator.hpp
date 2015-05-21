#ifndef CALIBRATOR_HPP_
#define CALIBRATOR_HPP_

#include "marg_data.hpp"

/**
 *
 */
enum class CalibratorState {
  uninit,
  idle,
  active
};

/**
 *
 */
class Calibrator {
public:
  void start(void);
  void stop(void);
  CalibratorState update(const marg_data_t &data);
private:
  CalibratorState state = CalibratorState::uninit;
  systime_t start_time;
  const systime_t timeout = MS2ST(200);
};

#endif /* CALIBRATOR_HPP_ */
