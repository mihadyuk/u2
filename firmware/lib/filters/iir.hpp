#ifndef IIR_HPP_
#define IIR_HPP_

#include <cmath>
#include <cstring>
#include <array>

#include "filter_base.hpp"

namespace filters {

/**
 *
 */
template <typename T, typename dataT, size_t L>
class IIR : public FilterBase<T, dataT> {
public:
  /**
   * @brief   Default constructor.
   */
  IIR(void) :
  a(nullptr),
  b(nullptr) {
    memset(a_state, 0, sizeof(a_state));
    memset(b_state, 0, sizeof(b_state));
  }

  /**
   * @brief   Default constructor.
   */
  IIR(const T *a_taps, const T *b_taps) :
  a(a_taps),
  b(b_taps) {
    osalDbgCheck((nullptr != a) && (nullptr != b));
    memset(a_state, 0, sizeof(a_state));
    memset(b_state, 0, sizeof(b_state));
  }

  /**
   * @brief   Switch transformation kernels.
   */
  void setKernel(const T *a_taps, const T *b_taps) {
    osalDbgCheck((nullptr != a_taps) && (nullptr != b_taps));
    a = a_taps;
    b = b_taps;
  }

  /**
   *
   */
  T update(dataT sample) {

    size_t i;
    T s;

    /* filter */
    s = sample * b[0];
    for (i=0; i<L; i++) {
      s += b_state[i] * b[i+1] + a_state[i] * a[i];
    }
    osalDbgCheck(! std::isinf(s));
    /* shift B */
    for (i=L-1; i>0; i--) {
      a_state[i] = a_state[i-1];
      b_state[i] = b_state[i-1];
    }
    b_state[0] = sample;
    a_state[0] = s;

    return s;
  }

private:
  const T *a;
  const T *b;
  dataT a_state[L];
  dataT b_state[L];
};

} /* namespace */

#endif /* IIR_HPP_ */
