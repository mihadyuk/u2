#ifndef DRIVETRAIN_IMPACT_HPP_
#define DRIVETRAIN_IMPACT_HPP_

#define DRIVETRAIN_IMPACT_CHANNELS   4

namespace Drive {

/**
 *
 */
typedef enum {
  IMPACT_ROLL = 0,
  IMPACT_PITCH = 1,
  IMPACT_YAW = 2,
  IMPACT_THRUST = 3,
} impact_ch_t;

/**
 * @brief   Output data from ACS
 */
typedef struct {
  float a[DRIVETRAIN_IMPACT_CHANNELS];    // normalized angle values -1..1
}DrivetrainImpact;

} /* namespace Drive */

#endif /* DRIVETRAIN_IMPACT_HPP_ */
