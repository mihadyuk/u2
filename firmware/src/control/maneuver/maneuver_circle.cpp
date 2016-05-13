#include <cmath>
#include "maneuver_circle.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"
#include "sign.hpp"

namespace control
{
namespace maneuver
{

enum class ApproachToCircle: uint32_t
{
  lineToCircleBorder,
  count
};

enum class CircleParts: uint32_t
{
  rightUpperArc,
  rightBottomArc,
  leftBottomArc,
  leftUpperArc,
  count
};

void circleManeuver(
    ManeuverPart &part,
    uint32_t partNumber,
    float repeats,
    float radius,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1])
{
  uint32_t partsCount = round(
        fabs(repeats)
      * static_cast<uint32_t>(CircleParts::count)
      + static_cast<uint32_t>(ApproachToCircle::count));

  mnrfp lineVector[2][1];
  m_minus<mnrfp, 2, 1>(lineVector, localPrev, localTrgt);
  mnrfp normedLineVector[2][1];
  m_copy<mnrfp, 2, 1>(normedLineVector, lineVector);
  m_norm<mnrfp, 2>(normedLineVector);

  if (   partNumber >= static_cast<uint32_t>(ApproachToCircle::count)
      && partNumber < partsCount)
  {
    /* quarter circle */
    mnrfp cwSign = sign(radius);
    mnrfp tangentCourse = tangentLineCourse(normedLineVector, cwSign);
    mnrfp startCourse = 0.0;

    CircleParts circlePart = static_cast<CircleParts>(
        (  partNumber
         - static_cast<uint32_t>(ApproachToCircle::count))
      % static_cast<uint32_t>(CircleParts::count));

    switch (circlePart)
    {
      case CircleParts::rightUpperArc:
        startCourse = tangentCourse;
        break;

      case CircleParts::rightBottomArc:
        startCourse = wrap_2pi(
            tangentCourse
          + cwSign*static_cast<mnrfp>(M_PI_2));
        break;

      case CircleParts::leftBottomArc:
        startCourse = wrap_2pi(
            tangentCourse
          + cwSign*static_cast<mnrfp>(M_PI));
        break;

      case CircleParts::leftUpperArc:
        startCourse = wrap_2pi(
            tangentCourse
          + cwSign*static_cast<mnrfp>(3.0 * M_PI_2));
        break;

      default:
        break;
    }

    part.fillArc(localTrgt, radius, startCourse, M_PI_2);

    if ((partsCount - 1) == partNumber)
      part.setFinal(true);
    else
      part.setFinal(false);
  }
  else if (partNumber < static_cast<uint32_t>(ApproachToCircle::count))
  {
    /* line from previous waypoint to the circle's border */
    mnrfp tmp[2][1];
    m_mul_s<mnrfp, 2, 1>(tmp, normedLineVector, fabs(radius));
    m_plus<mnrfp, 2, 1>(tmp, localTrgt, tmp);
    part.fillLine(localPrev, tmp);
    if ((partsCount - 1) == partNumber)
      part.setFinal(true);
    else
      part.setFinal(false);
  }
  else
  {
    /* out of maneuver parts range */
    part.fillUnknown();
  }
}

} /* namespace maneuver */
} /* namespace control */
