#ifndef ALCOI_HPP_
#define ALCOI_HPP_

#include "stabilizer/stabilizer.hpp"

namespace control {

struct AlcoiPulse {
  OverrideLevel lvl;
  pid_chain_t   ch;
  float         width; // seconds
  float         strength;
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
  bool ready = false;
  bool pulse_running = false;
  float time_elapsed = 0;
  AlcoiPulse pulse;
};

} /* namespace */

#endif /* ALCOI_HPP_ */
