#ifndef STABILIZER_HPP_
#define STABILIZER_HPP_

#include "drivetrain/drivetrain.hpp"
#include "state_vector.hpp"
#include "target_vector.hpp"
#include "pid.hpp"

namespace control {

/**
 *
 */
class Stabilizer {
public:
  Stabilizer(Drivetrain &drivetrain);
  void update(const TargetVector &trgt, const StateVector &state, float dT);
  void reset(void);
  void start(void);
  void stop(void);
  void dryRun(void);

protected:
  Drivetrain &drivetrain;
  PidControlSelfDerivative<float> pid_speed;
  PidControlSelfDerivative<float> pid_roll;
  PidControlSelfDerivative<float> pid_pitch;
  PidControlSelfDerivative<float> pid_yaw;
  bool ready;
};

} /* namespace */

#endif /* STABILIZER_HPP_ */
