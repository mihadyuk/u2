#ifndef PID_LINK_HPP_
#define PID_LINK_HPP_

#include <pid.hpp>

namespace control {

class PIDLink {
public:
  PIDLink(float const &position, PidControlSelfDerivative<float> &pid);
  void start(float const *pGain, float const *iGain, float const *dGain);
  void stop(void);
  float update(float target, float dT);
private:
  float const &position;
  PidControlSelfDerivative<float> &pid;
  bool ready;
};

} // namespace

#endif /* PID_LINK_HPP_ */
