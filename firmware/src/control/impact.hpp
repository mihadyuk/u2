#ifndef CONTROL_IMPACT_HPP_
#define CONTROL_IMPACT_HPP_

#define DRIVETRAIN_IMPACT_CHANNELS   4

namespace control {

/**
 *
 */
typedef enum {
  IMPACT_ROLL = 0,
  IMPACT_PITCH = 1,
  IMPACT_YAW = 2,
  IMPACT_SPEED = 3,
} impact_ch_t;

/**
 * @brief   Output data from ACS
 */
typedef struct {
  float a[DRIVETRAIN_IMPACT_CHANNELS];    // normalized angle values -1..1
}Impact;

} /* namespace */

#endif /* CONTROL_IMPACT_HPP_ */
