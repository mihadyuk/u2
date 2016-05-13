#include <cmath>
#include "maneuver_stadium.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"
#include "sign.hpp"

namespace control
{
namespace maneuver
{

enum class ApproachToStadium: uint32_t
{
  lineToStadiumBorder,
  count
};

enum class StadiumParts: uint32_t
{
  rightUpperLine,
  rightUpperArc,
  upperLine,
  leftUpperArc,
  leftLine,
  leftBottomArc,
  bottomLine,
  rightBottomArc,
  rightBottomLine,
  count
};

void stadiumManeuver(
    ManeuverPart &part,
    uint32_t partNumber,
    float repeats,
    float width,
    float height,
    float angle, /* rad */
    float radius,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1])
{
//  /* input parameters check */
//  if (radius < 0.0f)
//    radius = std::fabs(radius);

  uint32_t partsCount = std::round(
      std::fabs(repeats)
    * static_cast<uint32_t>(StadiumParts::count)
    + static_cast<uint32_t>(ApproachToStadium::count));

  mnrfp lineVector[2][1];
  m_minus<mnrfp, 2, 1>(lineVector, localPrev, localTrgt);
  mnrfp normedLineVector[2][1];
  m_copy<mnrfp, 2, 1>(normedLineVector, lineVector);
  m_norm<mnrfp, 2>(normedLineVector);

  if (   partNumber >= static_cast<uint32_t>(ApproachToStadium::count)
      && partNumber < partsCount)
  {
    /* maneuver parts */
    mnrfp northOffset = height / 2.0 - radius;
    mnrfp eastOffset = width / 2.0 - radius;
    mnrfp semiWidth = width / 2.0;
    mnrfp semiHeight = height / 2.0;

    StadiumParts stadiumPart = static_cast<StadiumParts>(
            (  partNumber
             - static_cast<uint32_t>(ApproachToStadium::count))
          % static_cast<uint32_t>(StadiumParts::count));

    switch (stadiumPart)
    {
      case StadiumParts::rightUpperLine:
        part.fillLine(
            0.0,
           -semiWidth,
            northOffset,
           -semiWidth);
        part.setFinal(false);
        break;

      case StadiumParts::rightUpperArc:
        part.fillArc(
            northOffset,
           -eastOffset,
            radius,
            0.0,
            M_PI_2);
        part.setFinal(false);
        break;

      case StadiumParts::upperLine:
        part.fillLine(
            semiHeight,
           -eastOffset,
            semiHeight,
            eastOffset);
        part.setFinal(false);
        break;

      case StadiumParts::leftUpperArc:
        part.fillArc(
            northOffset,
            eastOffset,
            radius,
            M_PI_2,
            M_PI_2);
        part.setFinal(false);
        break;

      case StadiumParts::leftLine:
        part.fillLine(
            northOffset,
            semiWidth,
           -northOffset,
            semiWidth);
        part.setFinal(false);
        break;

      case StadiumParts::leftBottomArc:
        part.fillArc(
           -northOffset,
            eastOffset,
            radius,
            M_PI,
            M_PI_2);
        part.setFinal(false);
        break;

      case StadiumParts::bottomLine:
        part.fillLine(
           -semiHeight,
            eastOffset,
           -semiHeight,
           -eastOffset);
        part.setFinal(false);
        break;

      case StadiumParts::rightBottomArc:
        part.fillArc(
           -northOffset,
           -eastOffset,
            radius,
            3.0 * M_PI_2,
            M_PI_2);
        part.setFinal(false);
        break;

      case StadiumParts::rightBottomLine:
        part.fillLine(
           -northOffset,
           -semiWidth,
            0.0,
           -semiWidth);

        part.setFinal(false);
        break;

      default:
        break;
    }

    if (sign(repeats) < 0.0)
      part.flipEast();

    part.rotate(angle);
    part.move(localTrgt);
  }
  else if (partNumber < static_cast<uint32_t>(ApproachToStadium::count))
  {
    /* line from previous waypoint to the stadium's border */
    part.fillLine(0.0, 0.0, 0.0, -width / 2.0);
    part.setFinal(false);
    part.rotate(angle);
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
