#ifndef CONTROL_DRIVETRAIN_IMPACT_HPP_
#define CONTROL_DRIVETRAIN_IMPACT_HPP_

namespace control {

/**
 * @brief     Output data from ACS
 * @details   Normalized values -1..1.
 */
struct DrivetrainImpact {
  float ail = 0;
  float ele = 0;
  float rud = 0;
  float thr = 0;
};

} /* namespace */

#endif /* CONTROL_DRIVETRAIN_IMPACT_HPP_ */
