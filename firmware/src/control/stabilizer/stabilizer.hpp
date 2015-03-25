#ifndef STABILIZER_HPP_
#define STABILIZER_HPP_

#include "pid_chain.hpp"
#include "state_vector.hpp"
#include "override_level_enum.hpp"
#include "drivetrain/drivetrain.hpp"
#include "pid_chain_enum.hpp"

namespace control {

/**
 *
 */
struct StabInput {
  ChainInput ch[PID_CHAIN_ENUM_END];
};

/**
 *
 */
struct pid_out {
  float ch[PID_CHAIN_ENUM_END];
};

/**
 *
 */
class Stabilizer {
public:
  Stabilizer(Drivetrain &drivetrain, const StateVector &s);
  void start(void);
  void update(const StabInput &in, float dT);
  void stop(void);
private:
  Drivetrain &drivetrain;
  bool ready = false;
  PIDChain ail_chain, ele_chain, rud_chain, thr_chain;
  PIDChain *chain[PID_CHAIN_ENUM_END];
};

} // namespace

#endif /* STABILIZER_HPP_ */
