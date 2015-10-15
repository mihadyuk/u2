#ifndef ADC_LOCAL_HPP_
#define ADC_LOCAL_HPP_

#include "alpha_beta.hpp"

class ADCLocal {
public:
  void start(void);
  void stop(void);
  adcsample_t getChannel(size_t N, filters::AlphaBetaBase<int32_t> &filter);
  adcsample_t getChannel(size_t N);
private:
  bool ready = false;
};

#endif /* ADC_LOCAL_HPP_ */

