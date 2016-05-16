#ifndef MANEUVER_THREE_POINTS_HPP_
#define MANEUVER_THREE_POINTS_HPP_

#include <stdint.h>
#include "maneuver_part.hpp"

namespace control
{
namespace maneuver
{

void threePointsManeuver(
    ManeuverPart &part,
    uint32_t partNumber,
    float radius,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1],
    const mnrfp (&localThird)[2][1]);

} /* namespace maneuver */
} /* namespace control */

#endif /* MANEUVER_THREE_POINTS_HPP_ */
