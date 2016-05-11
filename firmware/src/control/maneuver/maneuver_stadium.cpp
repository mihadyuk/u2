#include <math.h>
#include "maneuver_stadium.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"
#include "sign.hpp"

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
    const mnrfp (&localTrgt)[2][1])
{
  const uint32_t STADIUM_PARTS_COUNT = 9;
  const uint32_t APPROACH_PARTS_COUNT = 1;

  uint32_t partsCount = round(
      fabs(repeats)
    * STADIUM_PARTS_COUNT
    + APPROACH_PARTS_COUNT);

  mnrfp lineVector[2][1];
  m_minus<mnrfp, 2, 1>(lineVector, localPrev, localTrgt);
  mnrfp normedLineVector[2][1];
  m_copy<mnrfp, 2, 1>(normedLineVector, lineVector);
  m_norm<mnrfp, 2>(normedLineVector);

  if (   partNumber > 0
      && partNumber < partsCount)
  {
    /* maneuver parts */
    mnrfp northOffset = height / 2.0 - radius;
    mnrfp eastOffset = width / 2.0 - radius;
    mnrfp semiWidth = width / 2.0;
    mnrfp semiHeight = height / 2.0;

    switch (partNumber % 9)
    {
      case 1:
        part.fillLine(
            0.0,
           -semiWidth,
            northOffset,
           -semiWidth);
        part.setFinal(false);
        break;

      case 2:
        part.fillArc(
            northOffset,
           -eastOffset,
            radius,
            0.0,
            M_PI_2);
        part.setFinal(false);
        break;

      case 3:
        part.fillLine(
            semiHeight,
           -eastOffset,
            semiHeight,
            eastOffset);
        part.setFinal(false);
        break;

      case 4:
        part.fillArc(
            northOffset,
            eastOffset,
            radius,
            M_PI_2,
            M_PI_2);
        part.setFinal(false);
        break;

      case 5:
        part.fillLine(
            northOffset,
            semiWidth,
           -northOffset,
            semiWidth);
        part.setFinal(false);
        break;

      case 6:
        part.fillArc(
           -northOffset,
            eastOffset,
            radius,
            M_PI,
            M_PI_2);
        part.setFinal(false);
        break;

      case 7:
        part.fillLine(
           -semiHeight,
            eastOffset,
           -semiHeight,
           -eastOffset);
        part.setFinal(false);
        break;

      case 8:
        part.fillArc(
           -northOffset,
           -eastOffset,
            radius,
            3.0 * M_PI_2,
            M_PI_2);
        part.setFinal(false);
        break;

      case 0:
        part.fillLine(
           -northOffset,
           -semiWidth,
            0.0,
           -semiWidth);

        part.setFinal(false);
        break;
    }

    if (sign(repeats) < 0.0)
    {
      part.flipEast();
    }
    part.rotate(deg2rad<mnrfp>(angle));
    part.move(localTrgt);
  }
  else if (0 == partNumber)
  {
    /* line from previous waypoint to the stadium's border */
    part.fillLine(0.0, 0.0, 0.0, -width / 2.0);
    part.setFinal(false);
    part.rotate(deg2rad<mnrfp>(angle));
    part.move(localTrgt);
    ManeuverLine line = part.getLine();
    part.fillLine(localPrev, line.finish);

  }
//  else if ((partsCount - 1) == partNumber) {
//    /* line from the stadium's border to the stadium's center */
//    part.fillLine(0.0, -width/2.0, 0.0, 0.0);
//    part.setFinal(false);
//    part.rotate(deg2rad<T>(angle));
//    part.move(localTrgt);
//
//  }
  else
  {
    /* out of maneuver parts range */
    part.fillUnknown();
  }

}

} /* namespace maneuver */
} /* namespace control */
