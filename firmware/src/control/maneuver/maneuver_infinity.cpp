#include <cmath>
#include "maneuver_infinity.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"
#include "sign.hpp"

namespace control
{
namespace maneuver
{

enum class ApproachToInfinity: uint32_t
{
  lineToInfinityCenter,
  count
};

enum class InfinityParts: uint32_t
{
  rightUpperLine,
  rightUpperArc,
  leftUpperArc,
  leftUpperRightBottomLine,
  rightBottomArc,
  leftBottomArc,
  leftBottomLine,
  count
};

void infinityManeuver(
    ManeuverPart &part,
    uint32_t partNumber,
    float repeats,
    float width,
    float height,
    float angle,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1])
{
  uint32_t partsCount = round(
        fabs(repeats)
      * static_cast<uint32_t>(InfinityParts::count)
      + static_cast<uint32_t>(ApproachToInfinity::count));

  mnrfp radius = width / 2.0;

  if (   partNumber >= static_cast<uint32_t>(ApproachToInfinity::count)
      && partNumber < partsCount)
  {
    /* maneuver parts */
    mnrfp distToArcCenter = static_cast<mnrfp>(height / 2.0) - radius;
    mnrfp arm = sqrt(
        distToArcCenter * distToArcCenter
      - radius * radius);
    mnrfp alpha = asin(radius / distToArcCenter);

    InfinityParts infinityPart = static_cast<InfinityParts>(
        (  partNumber
         - static_cast<uint32_t>(ApproachToInfinity::count))
      % static_cast<uint32_t>(InfinityParts::count));

    switch (infinityPart)
    {
      case InfinityParts::rightUpperLine:
        part.fillLine(0.0, 0.0, arm, 0.0);
        part.setFinal(false);
        part.rotate(alpha);
        break;

      case InfinityParts::rightUpperArc:
        part.fillArc(
            distToArcCenter,
            0.0,
           -fabs(radius),
            wrap_2pi(alpha),
            static_cast<mnrfp>(M_PI_2) + alpha);
        part.setFinal(false);
        break;

      case InfinityParts::leftUpperArc:
        part.fillArc(
            distToArcCenter,
            0.0,
           -fabs(radius),
            3.0 * M_PI_2,
            static_cast<mnrfp>(M_PI_2) + alpha);
        part.setFinal(false);
        break;

      case InfinityParts::leftUpperRightBottomLine:
        part.fillLine(arm, 0.0, -arm, 0.0);
        part.setFinal(false);
        part.rotate(-alpha);
        break;

      case InfinityParts::rightBottomArc:
        part.fillArc(
           -distToArcCenter,
            0.0,
            fabs(radius),
            wrap_2pi(-alpha + static_cast<mnrfp>(M_PI)),
            static_cast<mnrfp>(M_PI_2) + alpha);
        part.setFinal(false);
        break;

      case InfinityParts::leftBottomArc:
        part.fillArc(
           -distToArcCenter,
            0.0,
            fabs(radius),
            3.0 * M_PI_2,
            static_cast<mnrfp>(M_PI_2) + alpha);
        part.setFinal(false);
        break;

      case InfinityParts::leftBottomLine:
        part.fillLine(-arm, 0.0, 0.0, 0.0);
        part.setFinal(false);
        part.rotate(alpha);
        break;

      default:
        break;
    }

    if (sign(repeats) < 0.0)
      part.flipNorth();

    part.rotate(deg2rad<mnrfp>(static_cast<mnrfp>(angle)));
    part.move(localTrgt);

    if ((partsCount - 1) == partNumber)
        part.setFinal(true);
  }
  else if (partNumber < static_cast<uint32_t>(ApproachToInfinity::count))
  {
    /* line from previous waypoint to the infinity's center */
    part.fillLine(localPrev, localTrgt);
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
