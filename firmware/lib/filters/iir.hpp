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
template <typename T, size_t L>
class IIR : public FilterBase<T> {
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
   * @brief   Switch transformation kernels.
   */
  void setKernel(const T *a_taps, const T *b_taps) {
    a = a_taps;
    b = b_taps;
  }

  /**
   * @brief   main filter function.
   * @details @b is kernel for input (left) branch, @ is kernel for
   *          output (right) branch. @a is shorter than @b by one element,
   *          some time this unused element represents as unity.
   */
  T update(T sample) {

    T s;

    /* filter */
    s = sample * b[0];
    for (size_t i=0; i<L; i++) {
      s += b_state[i] * b[i+1] + a_state[i] * a[i];
      /*-----------------------^
       Note: signe '+' here is for performance reasons. You need to
       premultiply a[] kernel by -1 in higher level if it calculated
       for case like y(n) = S(P) - S(Q) */
    }

    /* shift delay lines */
    for (size_t i=L-1; i>0; i--) {
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
  T a_state[L];
  T b_state[L];
};

/**
 *
 */
template<typename T, size_t L, size_t links>
class IIRChain {
  static_assert(links > 1, "Chain with single link is pointless");
public:
  /**
   *
   */
  T update(T sample) {
    T s = gain[0] * chain[0].update(sample);
    for (size_t i=1; i<links; i++) {
      s += gain[i] * chain[i].update(s);
    }
    return s;
  }

  /**
   *
   */
  void setGain(const T *gain_p) {
    gain = gain_p;
  }

private:
  IIR<T, L> chain[links];
  const T *gain;
};

} /* namespace */

#endif /* IIR_HPP_ */
