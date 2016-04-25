#ifndef MEDIAN_H_
#define MEDIAN_H_

#include "filter_base.hpp"

namespace filters {

/**
 *
 */
template<typename T, size_t N>
class Median : public FilterBase<T> {
public:
  /**
   * default constructor
   */
  Median(void) {
    /* check correctness of template parameters
     * 1) length must be odd
     * 2) lenght more than 3 is inefficient
     * 3) lenght more than 5 is totally inefficient */
    static_assert(((3 == N) || (5 == N)), "incorrect filter length");

    for(size_t i=0; i<N; i++) {
      buf[i] = 0;
    }
  };

  /**
   * Update filter state and return filtered value
   */
  T update (T sample) {
    size_t j;
    T tmp;

    /* place new sample in fifo */
    for(j=1; j<N; j++){
      buf[j-1] = buf[j];
    }
    buf[j-1] = sample;

    /* place data in temporal buffer */
    for(j=0; j<N; j++){
      sorted[j] = buf[j];
    }

    /* booble sort */
    size_t i;
    for(i=0; i<=(N-1); i++){
      for(j=i+1; j<N; j++){
        if(sorted[i] > sorted[j]){
          tmp = sorted[i];
          sorted[i] = sorted[j];
          sorted[j] = tmp;
        }
      }
    }

    return sorted[N/2]; /* middle of sorted buffer */
  };

private:
  /**
   * Buffer to store filter state between calls
   */
  T buf[N];

  /**
   * Temporal buffer for storing sorted results
   */
  T sorted[N];
};

} /* namespace */

#endif /* MEDIAN_H_ */
