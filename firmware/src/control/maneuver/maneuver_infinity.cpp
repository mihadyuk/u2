#include <math.h>
#include "maneuver_infinity.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"
#include "sign.hpp"

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
    const mnrfp (&localTrgt)[2][1])
{
  const uint32_t INFINITY_PARTS_COUNT = 7;
  const uint32_t APPROACH_PARTS_COUNT = 1;

  uint32_t partsCount = round(
      fabs(repeats)
    * INFINITY_PARTS_COUNT
    + APPROACH_PARTS_COUNT);

  if (0 == partNumber)
  {
    /* line from previous waypoint to the infinity's center */
    part.fillLine(localPrev, localTrgt);
    part.setFinal(false);
  }
  else if (partNumber > (partsCount - 1))
  {
    /* out of maneuver parts range */
    part.fillUnknown();
  }
  else
  {
    /* maneuver parts */
    mnrfp distToArcCenter = height / 2.0 - radius;
    mnrfp arm = sqrt(
        distToArcCenter
      * distToArcCenter
      - static_cast<mnrfp>(radius * radius));
    mnrfp alpha = asin(static_cast<mnrfp>(radius) / distToArcCenter);

    switch (partNumber % 7)
    {
      case 1:
        part.fillLine(0.0, 0.0, arm, 0.0);
        part.setFinal(false);
        part.rotate(alpha);
        break;

      case 2:
        part.fillArc(
            distToArcCenter,
            0.0,
            -fabs(radius),
            wrap_2pi(alpha),
            static_cast<mnrfp>(M_PI_2) + alpha);
        part.setFinal(false);
        break;

      case 3:
        part.fillArc(
            distToArcCenter,
            0.0,
            -fabs(radius),
            3.0 * M_PI_2,
            static_cast<mnrfp>(M_PI_2) + alpha);
        part.setFinal(false);
        break;

      case 4:
        part.fillLine(arm, 0.0, -arm, 0.0);
        part.setFinal(false);
        part.rotate(-alpha);
        break;

      case 5:
        part.fillArc(
            -distToArcCenter,
            0.0,
            fabs(radius),
            wrap_2pi(-alpha + static_cast<mnrfp>(M_PI)),
            static_cast<mnrfp>(M_PI_2) + alpha);
        part.setFinal(false);
        break;

      case 6:
        part.fillArc(
            -distToArcCenter,
            0.0,
            fabs(radius),
            3.0 * M_PI_2,
            static_cast<mnrfp>(M_PI_2) + alpha);
        part.setFinal(false);
        break;

      case 0:
        part.fillLine(-arm, 0.0, 0.0, 0.0);
        part.setFinal(false);
        part.rotate(alpha);
        break;
    }

    if (sign(repeats) < 0.0)
    {
      part.flipNorth();
    }

    part.rotate(deg2rad<mnrfp>(angle));
    part.move(localTrgt);

  }

  if ((partsCount - 1) == partNumber)
  {
    part.setFinal(true);
  }
}

} /* namespace maneuver */
} /* namespace control */
