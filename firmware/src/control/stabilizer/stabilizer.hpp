#ifndef STABILIZER_HPP_
#define STABILIZER_HPP_

#include <target_attitude.hpp>
#include "drivetrain/drivetrain.hpp"
#include "state_vector.hpp"
#include "futaba_data.hpp"
#include "pid.hpp"

namespace control {

/**
 *
 */
class Stabilizer {
public:
  Stabilizer(Drivetrain &drivetrain);
  void update(const FutabaData &futaba_data,
              const TargetAttitude &target_attitude,
              const StateVector &state,
              float dT);
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
