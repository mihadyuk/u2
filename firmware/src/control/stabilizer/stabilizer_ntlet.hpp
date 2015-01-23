#ifndef STABILIZER_PLANE_HPP_
#define STABILIZER_PLANE_HPP_

#include "stabilizer.hpp"
#include "pid.hpp"

class StabilizerPlane : public Stabilizer{
public:
  StabilizerPlane(Impact &impact, const StateVector &state_vector);
  void start(void);
  void stop(void);
  void sampleState(void);
  void dryRun(void);

private:
  void feng_shui(float *thrust_impact);
  void update_impl(float roll_trgt, float pitch_trgt, float speed_trgt);
  void reset_impl(void);
  PIDControlNG<float> pid_spd;
  PIDControlNG<float> pid_ail;
  PIDControlNG<float> pid_ele;
  PIDControlNG<float> pid_rud;
  float const *k_pitch;
  float const *thrust_min;
  float const *ele_sens, *ail_sens;
  float const *ele_lim, *ail_lim;
  float const *pitch_bal;
};

#endif /* STABILIZER_PLANE_HPP_ */
