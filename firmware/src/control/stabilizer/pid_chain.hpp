#ifndef PID_CHAIN_HPP_
#define PID_CHAIN_HPP_

#include "pid_link.hpp"
#include "override_level_enum.hpp"

namespace control {

struct ChainInput {
  /* regular input value passed to the top of chain */
  float target;
  /* value overriding PID input at specified level */
  float override_target;
  OverrideLevel override_level;
};



/**
 *
 */
class PIDChain {
public:
  PIDChain(const float &position_h,
           const float &position_m,
           const float &position_l,
           PidControlSelfDerivative<float> &pid_h,
           PidControlSelfDerivative<float> &pid_m,
           PidControlSelfDerivative<float> &pid_l);
  void start(const PIDInit<float> &h, const PIDInit<float> &m, const PIDInit<float> &l);
  float update(const ChainInput &in, float dT);
private:
  PIDLink link_h, link_m, link_l;
  float track_h = 0, track_m = 0, track_l = 0;
};

} // namespace

#endif /* PID_CHAIN_HPP_ */
