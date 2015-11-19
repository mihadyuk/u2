#ifndef ODOMETER_STM_HPP_
#define ODOMETER_STM_HPP_

#include "odometer.hpp"
#include "alpha_beta.hpp"
#include "median.hpp"

/**
 *
 */
enum class SampleCosher {
  yes,
  no
};

/**
 *
 */
class OdometerSTM : public Odometer {
private:
  void start_impl(void);
  void stop_impl(void);
  void update_impl(odometer_data_t &result, float dT);
  friend void odometer_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p);
  friend void odo_vtfunc(void *p);
  bool check_sample(uint32_t *path_ret, uint16_t *last_pulse_period, float dT);
  systime_t capture_time;
  uint32_t total_path_prev; /* for timeout detection */
  uint32_t new_sample_seq;
  static uint32_t total_path;
  static uint16_t period_cache;
  filters::AlphaBeta<float, 4> filter_alphabeta;
  filters::Median<float, 3> filter_median;
  SampleCosher sample_state;
};

#endif /* ODOMETER_STM_HPP_ */
