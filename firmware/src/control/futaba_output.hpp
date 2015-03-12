#ifndef FUTABA_OUTPUT_HPP_
#define FUTABA_OUTPUT_HPP_

#include "override_level.hpp"
#include "manual_switch.hpp"

namespace control {

/**
 * @brief   Output data from Futaba
 */
struct FutabaOutput {
  OverrideLevel ol_ail;
  OverrideLevel ol_ele;
  OverrideLevel ol_rud;
  OverrideLevel ol_thr;
  float ail = 0;
  float ele = 0;
  float rud = 0;
  float thr = 0;
  ManualSwitch man;
};

} /* namespace */

#endif /* FUTABA_OUTPUT_HPP_ */
