#ifndef STAB_VM_HPP_
#define STAB_VM_HPP_

#include "drivetrain/drivetrain.hpp"
#include "state_vector.hpp"
#include "mavlink_local.hpp"

namespace control {

/**
 * Command set
 */
typedef enum {
  END,
  INPUT,
  PID,
  SUM,
  SCALE,
  FORK,
  FORK_RET,
  OUTPUT,
  NEG,    /* value negation */
  TERM    /* terminator */
} vm_opcode_enum;

/**
 *
 */
class StabVM {
public:
  StabVM(DrivetrainImpact &impact, const StateVector &sv);
  void start(void);
  void stop(void);
  void update(float dT, const uint8_t *bytecode);
  bool verify(const uint8_t *bytecode);
private:
  void pid_pool_start(void);
  void scale_pool_start(void);
  void exec(void);
  void compile(const uint8_t *bytecode);
  void destroy(void);

  DrivetrainImpact &impact;
  const StateVector &sv;
  bool ready = false;
  time_measurement_t exec_tmo;
  const uint8_t *current_program = nullptr;
};

} /* namespace */

#endif /* STAB_VM_HPP_ */
