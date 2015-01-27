#ifndef CONTROL_DIRECTION_HPP_
#define CONTROL_DIRECTION_HPP_

namespace control {

/**
 *
 */
typedef enum {
  DIRECTION_CH_COURSE,     // rad
  DIRECTION_CH_HEIGHT,     // m
  DIRECTION_CH_SPEED,      // m/s
  DIRECTION_CH_ENUM_END
} direction_ch_t;

/**
 * @brief   Output data from ACS
 */
struct TargetDirection {
  uint32_t mask = 0;
  float a[DIRECTION_CH_ENUM_END];
};

} /* namespace */

#endif /* CONTROL_DIRECTION_HPP_ */
