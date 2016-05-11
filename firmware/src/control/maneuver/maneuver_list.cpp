#include <cmath>
#include "maneuver_list.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"

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
    const mnrfp (&localTrgt)[2][1])
{
  part.fillLine(localPrev, localTrgt);
  part.setFinal(true);
}

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
    mnrfp trgtAlt)
{
  part.fillLine(localPrev, localTrgt);
  part.setFinal(true);

  /* calculate target altitude for current position */
  mnrfp deltaAlt = trgtAlt - prevAlt;
  mnrfp prevToTrgtVector[2][1];
  m_minus<mnrfp, 2, 1>(prevToTrgtVector, localTrgt, localPrev);
  mnrfp prevToTrgtDistance = m_vec_norm<mnrfp, 2>(prevToTrgtVector);
  mnrfp slopeAngleTangent = deltaAlt / prevToTrgtDistance;
  mnrfp prevToCurrDistance = m_vec_norm<mnrfp, 2>(localPrev);
  mnrfp alt = prevAlt + prevToCurrDistance * slopeAngleTangent;
  part.fillAlt(alt);
}

/*
 * Maneuver for alignment
 * (altitude and cross track error minimize) before landing.
 */
void landingAlignmentManeuver(
    ManeuverPart &part,
    uint32_t partNumber,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1],
    mnrfp prevAlt,
    mnrfp trgtAlt)
{
  const float HEIGHT = 4.0f;
  const float WIDTH = 2.0f;

  /* calculate rotate angle for alignment arm of
   * infinity maneuver and direction to maneuver center */
  mnrfp distanceToArcCenter = (HEIGHT - WIDTH) / 2.0;
  mnrfp armRatationAngleSin =
      static_cast<mnrfp>(WIDTH / 2.0) / distanceToArcCenter;

  mnrfp armRatationAngle;
  if (0 != std::isinf(armRatationAngleSin))
    armRatationAngle = M_PI_2;
  else
    armRatationAngle = asin(armRatationAngleSin);

  mnrfp prevToTrgtVector[2][1];
  m_minus<mnrfp, 2, 1>(prevToTrgtVector, localTrgt, localPrev);

  mnrfp prevToTrgtCourse = atan2(
      prevToTrgtVector[1][0],
      prevToTrgtVector[0][0]);

  mnrfp rotateAngle = prevToTrgtCourse - armRatationAngle;

  infinityManeuver(
      part,
      partNumber,
      1.0f,
      WIDTH,
      HEIGHT,
      rotateAngle,
      localPrev,
      localTrgt);

  /* set target altitude after line from previous waypoint to the infinity center */
  if (partNumber > 0)
    part.fillAlt(trgtAlt);
  else
    part.fillAlt(prevAlt);
}

} /* namespace maneuver */
} /* namespace control */
