#ifndef CONTROL_ACS_HPP_
#define CONTROL_ACS_HPP_

#include "stabilizer/stabilizer.hpp"
#include "futaba/futaba.hpp"
#include "state_vector.hpp"

namespace control {

/**
 *
 */
class ACS {
public:
  ACS(Drivetrain &drivetrain, const StateVector &s);
  void start(void);
  void update(float dT);
  void stop(void);
private:
  void failsafe(void);
  Futaba futaba;
  Stabilizer stabilizer;
  bool ready = false;
  bool ignore_futaba_fail = false;
};

} // namespace

#endif /* CONTROL_ACS_HPP_ */
