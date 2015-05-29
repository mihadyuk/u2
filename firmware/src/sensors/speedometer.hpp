#ifndef SPEEDOMETER_HPP_
#define SPEEDOMETER_HPP_

#include "speedometer_data.hpp"
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
class Speedometer {
public:
  void start(void);
  void stop(void);
  void update(speedometer_data_t &result, float dT);
private:
  friend void speedometer_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p);
  bool check_sample(uint32_t &path_ret, uint16_t &last_pulse_period, float dT);
  void speed2mavlink(const speedometer_data_t &result);
  systime_t capture_time;
  uint32_t total_path_prev; /* for timeout detection */
  uint32_t new_sample_seq;
  static uint32_t total_path;
  static uint16_t period_cache;
  filters::AlphaBeta<float, 4> filter_alphabeta;
  filters::Median<float, 3> filter_median;
  const float *pulse2m = nullptr;
  SampleCosher sample_state;
  bool ready = false;
};

#endif /* SPEEDOMETER_HPP_ */
