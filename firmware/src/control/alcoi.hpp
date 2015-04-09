#ifndef ALCOI_HPP_
#define ALCOI_HPP_

#include "mavlink_local.hpp"
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
  enum MAV_RESULT commandHandler(const mavlink_command_long_t *clp);
  void update(StabInput &stab, float dT);
private:
  bool load_pulse(const AlcoiPulse &pulse);
  AlcoiState state = AlcoiState::uninit;
  float pulse_time_elapsed = 0;
  float relax_time_elapsed = 0;
  AlcoiPulse pulse;
};

} /* namespace */

#endif /* ALCOI_HPP_ */
