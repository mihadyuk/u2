#ifndef FUTABA_OUTPUT_HPP_
#define FUTABA_OUTPUT_HPP_

#include "override_level_enum.hpp"
#include "manual_switch_enum.hpp"

#define MAX_FUTABA_CHANNELS                   8

namespace control {

/**
 * @brief   Output data from Futaba
 */
struct FutabaOutput {

  float ch[MAX_FUTABA_CHANNELS];
  /**
   * @brief   Manual switch state
   */
  ManualSwitch man = ManualSwitch::manual;
};

} /* namespace */

#endif /* FUTABA_OUTPUT_HPP_ */
