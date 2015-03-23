#ifndef CONTROL_DRIVETRAIN_IMPACT_HPP_
#define CONTROL_DRIVETRAIN_IMPACT_HPP_

#include "pid_chain_enum.hpp"

namespace control {

/**
 * @brief     Output data from ACS
 * @details   Normalized values -1..1.
 */
struct DrivetrainImpact {
  float ch[PID_CHAIN_ENUM_END];
};

} /* namespace */

#endif /* CONTROL_DRIVETRAIN_IMPACT_HPP_ */
