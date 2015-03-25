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
  void start(float const *pGain_h, float const *iGain_h, float const *dGain_h, uint32_t const *bypass_h,
             float const *pGain_m, float const *iGain_m, float const *dGain_m, uint32_t const *bypass_m,
             float const *pGain_l, float const *iGain_l, float const *dGain_l, uint32_t const *bypass_l);
  float update(const ChainInput &in, float dT);
private:
  PIDLink link_h, link_m, link_l;
  float track_h = 0, track_m = 0, track_l = 0;
};

} // namespace

#endif /* PID_CHAIN_HPP_ */
