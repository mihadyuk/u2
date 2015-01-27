#ifndef CONTROL_TARGET_ATTITUDE_HPP_
#define CONTROL_TARGET_ATTITUDE_HPP_

namespace control {

/**
 *
 */
typedef enum {
  ATTITUDE_CH_ROLL,     // rad
  ATTITUDE_CH_PITCH,    // rad
  ATTITUDE_CH_YAW,      // rad
  ATTITUDE_CH_SPEED,    // m/s
  ATTITUDE_CH_ENUM_END
} attitude_ch_t;

/**
 * @brief   Output data from ACS
 */
struct TargetAttitude {
  uint32_t mask = 0;
  float a[ATTITUDE_CH_ENUM_END];
};

} /* namespace */

#endif /* CONTROL_TARGET_ATTITUDE_HPP_ */
