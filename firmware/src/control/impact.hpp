#ifndef CONTROL_IMPACT_HPP_
#define CONTROL_IMPACT_HPP_

namespace control {

/**
 *
 */
typedef enum {
  IMPACT_CH_ROLL,
  IMPACT_CH_PITCH,
  IMPACT_CH_YAW,
  IMPACT_CH_SPEED,
  IMPACT_CH_ENUM_END,
} impact_ch_t;

/**
 * @brief   Output data from ACS
 */
struct Impact {
  uint32_t mask = 0;
  float a[IMPACT_CH_ENUM_END];    // normalized angle values -1..1
};

} /* namespace */

#endif /* CONTROL_IMPACT_HPP_ */
