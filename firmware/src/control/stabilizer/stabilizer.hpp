#ifndef STABILIZER_HPP_
#define STABILIZER_HPP_

#include "drivetrain/drivetrain.hpp"
#include "state_vector.hpp"
#include "stabilizer_target_vector.hpp"
#include "pid.hpp"

/**
 *
 */
class Stabilizer {
public:
  Stabilizer(Control::Drivetrain &drivetrain);
  void update(const StabilizerTargetVector &trgt, const StateVector &state, float dT);
  void reset(void);
  void start(void);
  void stop(void);
  void dryRun(void);

protected:
  Control::Drivetrain &drivetrain;
  PidControlSelfDerivative<float> pid_speed;
  PidControlSelfDerivative<float> pid_roll;
  PidControlSelfDerivative<float> pid_pitch;
  PidControlSelfDerivative<float> pid_yaw;
  bool ready;
};

#endif /* STABILIZER_HPP_ */
