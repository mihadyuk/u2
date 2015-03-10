#ifndef PID_CHAIN_HPP_
#define PID_CHAIN_HPP_

#include "pid_link.hpp"
#include "override_level.hpp"

namespace control {

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
  void start(float const *pGain_h, float const *iGain_h, float const *dGain_h,
             float const *pGain_m, float const *iGain_m, float const *dGain_m,
             float const *pGain_l, float const *iGain_l, float const *dGain_l);
  float update(float target, float dT, OverrideLevel ol);
private:
  PIDLink link_h, link_m, link_l;
};

} // namespace

#endif /* PID_CHAIN_HPP_ */
