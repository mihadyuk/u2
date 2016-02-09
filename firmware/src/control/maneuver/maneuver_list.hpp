#ifndef MANEUVER_LIST_HPP_
#define MANEUVER_LIST_HPP_

#include <math.h>
//#include "mavlink_local.hpp"
#include "maneuver_part.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"
//#include "maneuver_utils.hpp"
#include "maneuver_circle.hpp"
#include "maneuver_three_points.hpp"
#include "maneuver_infinity.hpp"
#include "maneuver_stadium.hpp"

namespace control {
namespace maneuver {

/*
 * Simple line between two points in local (north and east) coordinates.
 */
template <typename T>
void lineManeuver(ManeuverPart<T> &part,
                  const T (&localPrev)[2][1],
                  const T (&localTrgt)[2][1]) {

  part.fillLine(localPrev, localTrgt);
  part.setFinal(true);
}

/*
 * Simple line between two points in local (north and east) coordinates.
 * Altitude is directly proportional the distance from previous point
 * (in range [prevAlt; trgtAlt]).
 */
template <typename T>
void lineSlopeManeuver(ManeuverPart<T> &part,
                       const T (&localPrev)[2][1],
                       const T (&localTrgt)[2][1],
                       T prevAlt,
                       T trgtAlt) {

  part.fillLine(localPrev, localTrgt);
  part.setFinal(true);

  /* calculate target altitude for current position */
  T deltaAlt = trgtAlt - prevAlt;
  T prevToTrgtVector[2][1];
  m_minus<T, 2, 1>(prevToTrgtVector, localTrgt, localPrev);
  T prevToTrgtDistance = m_vec_norm<T, 2>(prevToTrgtVector);
  T slopeAngleTangent = deltaAlt/prevToTrgtDistance;
  T prevToCurrDistance = m_vec_norm<T, 2>(localPrev);
  T alt = prevAlt + prevToCurrDistance*slopeAngleTangent;

  part.fillAlt(alt);
}

/*
 * Maneuver for alignment (altitude and cross track error minimize) before landing.
 */
template <typename T>
void landingAlignmentManeuver(ManeuverPart<T> &part,
                              uint32_t partNumber,
                              const T (&localPrev)[2][1],
                              const T (&localTrgt)[2][1],
                              T prevAlt,
                              T trgtAlt) {

  const float HEIGHT = 6.0f;
  const float RADIUS = 2.0f;

  /* calculate rotate angle for alignment arm of infinity maneuver and direction to maneuver center */
  T distanceToArcCenter = HEIGHT/2.0 - RADIUS; /* if maneuver center in the coordinates 0, 0 */
  T armRatationAngle = asin(static_cast<T>(RADIUS)/distanceToArcCenter);
  T prevToTrgtVector[2][1];
  m_minus<T, 2, 1>(prevToTrgtVector, localTrgt, localPrev);
  T prevToTrgtCourse = atan2(prevToTrgtVector[1][0],
                             prevToTrgtVector[0][0]);
  T rotateAngle = prevToTrgtCourse - armRatationAngle;

  infinityManeuver(part,
                   partNumber,
                   1.0f,
                   RADIUS,
                   HEIGHT,
                   rotateAngle,
                   localPrev,
                   localTrgt);

  /* set target altitude after line from previous waypoint to the infinity center */
  if (partNumber > 0)
    part.fillAlt(prevAlt);
  else
    part.fillAlt(trgtAlt);

}



} /* namespace maneuver */
} /* namespace control */

#endif /* MANEUVER_LIST_HPP_ */
