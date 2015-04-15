#ifndef VM2_HPP_
#define VM2_HPP_

#include "mavlink_local.hpp"
#include "stabilizer/stabilizer.hpp"

namespace control {

class VM2 {
public:
  void start(void);
  void stop(void);
  void update(float dT);
private:
  void pid_pool_start(void);
  void scale_pool_start(void);
  void exec(void);
  void compile(const uint8_t *bytecode);
  void destroy_chain(void);
  bool ready = false;
};

} /* namespace */

#endif /* VM2_HPP_ */
