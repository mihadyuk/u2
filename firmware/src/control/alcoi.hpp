#ifndef ALCOI_HPP_
#define ALCOI_HPP_

#include "stabilizer/stabilizer.hpp"

namespace control {

/**
 *
 */
struct AlcoiPulse {
  OverrideLevel lvl;
  pid_chain_t   ch;
  float         width; // seconds
  float         strength;
};

/**
 *
 */
enum class AlcoiState {
  uninit,
  idle,
  pulse,
  relax
};

/**
 *
 */
class Alcoi {
public:
  void start(void);
  void stop(void);
  bool loadPulse(const AlcoiPulse &pulse);
  void update(StabInput &stab, float dT);
private:
  AlcoiState state = AlcoiState::uninit;
  float pulse_time_elapsed = 0;
  float relax_time_elapsed = 0;
  AlcoiPulse pulse;
};

} /* namespace */

#endif /* ALCOI_HPP_ */
