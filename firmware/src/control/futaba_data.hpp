#ifndef FUTABA_DATA_HPP_
#define FUTABA_DATA_HPP_

#include "override_level.hpp"
#include "manual_switch.hpp"

namespace control {

/**
 * @brief   Output data from Futaba
 */
typedef struct {
  OverrideLevel level;
  ManualSwitch man;
  float ail = 0;
  float ele = 0;
  float rud = 0;
  float thr = 0;
} FutabaData;

} /* namespace */

#endif /* FUTABA_DATA_HPP_ */
