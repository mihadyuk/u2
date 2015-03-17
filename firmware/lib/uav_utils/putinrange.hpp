#ifndef PUTINRANGE_HPP_
#define PUTINRANGE_HPP_

#include "min_max.hpp"

/**
 * Clamper function implementation
 */
template<typename T>
T __putinrange_impl(const T& v, const T& lower, const T& upper) {

  if (lower <= upper)
    return max(lower, min(v, upper));
  else /* protection from stupidity */
    return min(lower, max(v, upper));
}

/**
 * Clamper function wrapper
 */
template<typename T, typename T2, typename T3>
T putinrange(const T& v, const T2& lower, const T3& upper) {
  return __putinrange_impl(v, static_cast<T>(lower), static_cast<T>(upper));
}

#endif /* PUTINRANGE_HPP_ */
