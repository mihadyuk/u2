#ifndef CONTROL_HIL_HPP_
#define CONTROL_HIL_HPP_

#include <bitset>

namespace control {

/**
 *
 */
class HIL {
public:
  HIL(void);
  void update(ACSInput &acs_in);
  void override(double val, state_vector_enum addr);
  void disable(state_vector_enum addr);
  void disableAll(void);
private:
  ACSInput shadow;
  std::bitset<ACS_INPUT_ENUM_END> bmp;
};

} /* namespace */

#endif /* CONTROL_HIL_HPP_ */

