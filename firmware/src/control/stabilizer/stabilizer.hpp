#ifndef STABILIZER_HPP_
#define STABILIZER_HPP_

#include "pid_chain.hpp"
#include "state_vector.hpp"
#include "override_level.hpp"
#include "drivetrain/drivetrain.hpp"

namespace control {

/**
 *
 */
struct pid_in {
  OverrideLevel ol_ail;
  OverrideLevel ol_ele;
  OverrideLevel ol_rud;
  OverrideLevel ol_thr;
  float ail;
  float ele;
  float rud;
  float thr;
  float dT;
};

/**
 *
 */
struct pid_out {
  float ail;
  float ele;
  float rud;
  float thr;
};

/**
 *
 */
class Stabilizer {
public:
  Stabilizer(Drivetrain &drivetrain, const StateVector &s);
  void start(void);
  void update(const pid_in &in);
  void stop(void);
private:
  Drivetrain &drivetrain;
  bool ready = false;
  PIDChain ail_chain, ele_chain, rud_chain, thr_chain;
};

} // namespace

#endif /* STABILIZER_HPP_ */
