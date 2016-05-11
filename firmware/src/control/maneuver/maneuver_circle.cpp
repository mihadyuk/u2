#include <cmath>
#include "maneuver_circle.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"
#include "sign.hpp"

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
    const mnrfp (&localTrgt)[2][1])
{

  const uint32_t CIRCLE_PARTS_COUNT = 4;
  const uint32_t APPROACH_PARTS_COUNT = 1;

  uint32_t partsCount = round(
        fabs(repeats)
      * CIRCLE_PARTS_COUNT
      + APPROACH_PARTS_COUNT);

  mnrfp lineVector[2][1];
  m_minus<mnrfp, 2, 1>(lineVector, localPrev, localTrgt);

  mnrfp normedLineVector[2][1];
  m_copy<mnrfp, 2, 1>(normedLineVector, lineVector);
  m_norm<mnrfp, 2>(normedLineVector);

  if (   partNumber > 0
      && partNumber < partsCount)
  {
    /* quarter circle */
    mnrfp cwSign = sign(radius);
    mnrfp tangentCourse = tangentLineCourse(normedLineVector, cwSign);
    mnrfp startCourse = 0.0;

    switch (partNumber % 4)
    {
      case 1:
        startCourse = tangentCourse;
        break;

      case 2:
        startCourse = wrap_2pi(
            tangentCourse
          + cwSign*static_cast<mnrfp>(M_PI_2));
        break;

      case 3:
        startCourse = wrap_2pi(
            tangentCourse
          + cwSign*static_cast<mnrfp>(M_PI));
        break;

      case 0:
        startCourse = wrap_2pi(
            tangentCourse
          + cwSign*static_cast<mnrfp>(3.0 * M_PI_2));
        break;
    }

    part.fillArc(localTrgt, radius, startCourse, M_PI_2);

    if ((partsCount - 1) == partNumber)
    {
      part.setFinal(true);
    }
    else
    {
      part.setFinal(false);
    }

  }
  else if (0 == partNumber)
  {
    /* line from previous waypoint to the circle's border */
    mnrfp tmp[2][1];
    m_mul_s<mnrfp, 2, 1>(tmp, normedLineVector, fabs(radius));
    m_plus<mnrfp, 2, 1>(tmp, localTrgt, tmp);

    part.fillLine(localPrev, tmp);
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
