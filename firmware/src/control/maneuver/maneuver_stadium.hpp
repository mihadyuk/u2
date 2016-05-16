#ifndef MANEUVER_STADIUM_HPP_
#define MANEUVER_STADIUM_HPP_

#include <stdint.h>
#include "maneuver_part.hpp"

namespace control
{
namespace maneuver
{

void stadiumManeuver(
    ManeuverPart &part,
    uint32_t partNumber,
    float repeats,
    float width,
    float height,
    float angle,
    float radius,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1]);

} /* namespace maneuver */
} /* namespace control */


#endif /* MANEUVER_STADIUM_HPP_ */
