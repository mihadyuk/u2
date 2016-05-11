#ifndef MANEUVER_CIRCLE_HPP_
#define MANEUVER_CIRCLE_HPP_

#include <stdint.h>
#include "maneuver_part.hpp"

namespace control
{
namespace maneuver
{

void circleManeuver(
    ManeuverPart &part,
    uint32_t partNumber,
    float repeats,
    float radius,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1]);

} /* namespace maneuver */
} /* namespace control */

#endif /* MANEUVER_CIRCLE_HPP_ */
