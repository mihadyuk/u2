#include <array>
#include <functional>

#include "hal.h"

/**
 * @brief   stub for std::function
 */
void std::__throw_bad_function_call(void) {
  osalSysHalt("__throw_bad_function_call");
  while(true);
}

/**
 * @brief   stub for std::array
 */
void std::__throw_out_of_range_fmt(char const*, ...) {
  osalSysHalt("__throw_out_of_range_fmt");
  while(true);
}

