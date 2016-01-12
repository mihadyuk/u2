#ifndef STAB_VM_HPP_
#define STAB_VM_HPP_

#include "acs_input.hpp"
#include "drivetrain/drivetrain.hpp"
#include "mavlink_local.hpp"

namespace control {

/**
 *
 */
struct AlcoiPulse {
  size_t  pid = 0xFF;
  float   width = 0; // seconds
  float   strength = 0; // context dependent
};

/**
 * Command set
 */
typedef enum {
  END,
  INPUT,
  PID,    /* usage: PID, №, curren_position. Target will be passed implicitly */
  SUM,
  SCALE,
  FORK,
  FORK_RET,
  OUTPUT,
  NEG,    /* value negation. It is simpler than scale by -1 */
  TERM    /* every branch must be ended with terminator */
} vm_opcode_enum;

/**
 *
 */
class StabVM {
public:
  StabVM(DrivetrainImpact &impact, const ACSInput &acs_in);
  void start(void);
  void stop(void);
  void update(float dT, const uint8_t *bytecode);
  bool alcoiPulse(const AlcoiPulse &pulse);
  bool verify(const uint8_t *bytecode);
private:
  void pid_pool_start(void);
  void scale_pool_start(void);
  void exec(void);
  void compile(const uint8_t *bytecode);
  void destroy(void);

  DrivetrainImpact &impact;
  const ACSInput &acs_in;
  float dT; /* needs only for PIDs */
  bool ready = false;
  time_measurement_t exec_tmo;
  const uint8_t *current_program = nullptr;
};

} /* namespace */

#endif /* STAB_VM_HPP_ */
