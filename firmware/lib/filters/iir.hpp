#ifndef IIR_HPP_
#define IIR_HPP_

#include <cstring>

namespace filters {

template <typename T, typename dataT, int L>
class IIR {
public:
  /**
   * @brief   Default constructor.
   */
  IIR(T *a_taps, T *b_taps) {

    static_assert(L > 0, "Zero size forbidden.");

    a = a_taps;
    b = b_taps;
    memset(a_state, 0, sizeof(a_state));
    memset(b_state, 0, sizeof(b_state));
  }

  /**
   *
   */
  T update(T sample) {

    size_t i;
    T s;

    /* filter */
    s = sample * b[0];
    for (i=0; i<L; i++)
      s += b_state[i] * b[i+1] + a_state[i] * a[i];

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
  T *a;
  T *b;
  dataT a_state[L];
  dataT b_state[L];
};

} /* namespace */

#endif /* IIR_HPP_ */
