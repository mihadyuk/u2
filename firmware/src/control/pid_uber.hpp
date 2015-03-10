#ifndef PID_UBER_HPP_
#define PID_UBER_HPP_

#include "pid_chain.hpp"
#include "state_vector.hpp"

namespace control {

/**
 *
 */
struct piduber_in {
  PIDChainOverrideLevel ol_ail;
  PIDChainOverrideLevel ol_ele;
  PIDChainOverrideLevel ol_rud;
  PIDChainOverrideLevel ol_thr;
  float ail;
  float ele;
  float rud;
  float thr;
  float dT;
};

/**
 *
 */
struct piduber_out {
  float ail;
  float ele;
  float rud;
  float thr;
};

/**
 *
 */
class PIDUber {
public:
  PIDUber(const StateVector &s);
  void start(void);
  void update(const piduber_in &in, piduber_out &out);
  void stop(void);
private:
  bool ready = false;
  PIDChain ail, ele, rud, thr;
};

} // namespace

#endif /* PID_UBER_HPP_ */
