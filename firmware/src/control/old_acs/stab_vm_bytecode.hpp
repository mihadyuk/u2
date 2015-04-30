#ifndef STAB_VM_HPP_
#define STAB_VM_HPP_

#include "mavlink_local.hpp"
#include "stabilizer/stabilizer.hpp"

namespace control {

class StabVM {
public:
  void start(void);
  void stop(void);
  void update(float dT);
  void exec(const uint8_t *program);
};

} /* namespace */

#endif /* STAB_VM_HPP_ */
