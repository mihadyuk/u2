#ifndef MANEUVER_INFINITY_HPP_
#define MANEUVER_INFINITY_HPP_

#include <stdint.h>
#include "maneuver_part.hpp"

namespace control
{
namespace maneuver
{

void infinityManeuver(
    ManeuverPart &part,
    uint32_t partNumber,
    float repeats,
    float radius,
    float height,
    float angle,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1]);

} /* namespace maneuver */
} /* namespace control */


#endif /* MANEUVER_INFINITY_HPP_ */
