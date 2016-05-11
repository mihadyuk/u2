#ifndef MANEUVER_LIST_HPP_
#define MANEUVER_LIST_HPP_

#include "maneuver_part.hpp"
#include "maneuver_circle.hpp"
#include "maneuver_three_points.hpp"
#include "maneuver_infinity.hpp"
#include "maneuver_stadium.hpp"

namespace control
{
namespace maneuver
{

/*
 * Simple line between two points in local (north and east) coordinates.
 */
void lineManeuver(
    ManeuverPart &part,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1]);

/*
 * Simple line between two points in local (north and east) coordinates.
 * Altitude is directly proportional the distance from previous point
 * (in range [prevAlt; trgtAlt]).
 */
void lineSlopeManeuver(
    ManeuverPart &part,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1],
    mnrfp prevAlt,
    mnrfp trgtAlt);

/*
 * Maneuver for alignment (altitude and cross track error minimize) before landing.
 */
void landingAlignmentManeuver(
    ManeuverPart &part,
    uint32_t partNumber,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1],
    mnrfp prevAlt,
    mnrfp trgtAlt);


} /* namespace maneuver */
} /* namespace control */

#endif /* MANEUVER_LIST_HPP_ */
