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
  void exec(void);
};

} /* namespace */

#endif /* VM2_HPP_ */
