#ifndef STABILIZER_ROVER_HPP_
#define STABILIZER_ROVER_HPP_

#include "stabilizer.hpp"
#include "pid.hpp"

class StabilizerRover : public Stabilizer{
public:
  StabilizerRover(Drive::DrivetrainImpact &impact, const StateVector &state_vector);
  void stop(void);
  void sampleState(void);
  void dryRun(void);

private:
  void start_impl(void);
  void update_impl(float roll_trgt, float pitch_trgt, float speed_trgt);
  void reset_impl(void);
  float const *k_pitch, *pitch_bal;
  float const *thrust_min;
  float const *ele_sens, *ail_sens;
  uint32_t const *loop_on;
};

#endif /* STABILIZER_ROVER_HPP_ */
