#ifndef ALCOI_HPP_
#define ALCOI_HPP_

#include "futaba_output.hpp"

namespace control {

/**
 *
 */
class Alcoi {
public:
  void start(void);
  void stop(void);
  msg_t update(FutabaOutput &result, float dT);
private:
  bool ready = false;
};

} /* namespace */

#endif /* ALCOI_HPP_ */
