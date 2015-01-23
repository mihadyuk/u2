#ifndef CONNECT_CHECKER_HPP_
#define CONNECT_CHECKER_HPP_

#include "alpha_beta.hpp"
#include "hysteresis.hpp"

/**
 *
 */
class ConnectChecker {
public:
  ConnectChecker(void) {
    return;
  }

  bool good(bool data_acquired, uint32_t timeout) {
    uint32_t goodness = 0;
    uint32_t v = 0;
    (void)timeout;

    if (CH_SUCCESS == data_acquired)
      v = 1000 * 2;  // 2 потому что эта функция вызывается в 2 раза чаще, чем приходят пакеты из псшима
    else
      v = 0;
    goodness = filter.update(v, 256);

    return hys.check(goodness);
  }

  bool good(void){
    return hys.check();
  }

private:
  AlphaBeta<uint32_t> filter;
  HysteresisBool<uint32_t, 700, 900, CH_FAILED> hys;
};

#endif /* CONNECT_CHECKER_HPP_ */
