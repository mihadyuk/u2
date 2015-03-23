#ifndef FUTABA_OUTPUT_HPP_
#define FUTABA_OUTPUT_HPP_

#include "override_level_enum.hpp"
#include "manual_switch_enum.hpp"
#include "pid_chain_enum.hpp"

namespace control {

/**
 * @brief   Output data from Futaba
 */
struct FutabaOutput {
  OverrideLevel ol[PID_CHAIN_ENUM_END];
  float ch[PID_CHAIN_ENUM_END];
  ManualSwitch man;
};

} /* namespace */

#endif /* FUTABA_OUTPUT_HPP_ */
