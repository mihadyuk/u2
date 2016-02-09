#ifndef MANEUVER_STADIUM_HPP_
#define MANEUVER_STADIUM_HPP_

#include <math.h>
#include "maneuver_part.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"

namespace control {
namespace maneuver {

template <typename T>
void stadiumManeuver(ManeuverPart<T> &part,
                     uint32_t partNumber,
                     float repeats,
                     float width,
                     float height,
                     float angle,
                     float radius,
                     const T (&localPrev)[2][1],
                     const T (&localTrgt)[2][1]) {

  uint32_t partsCount = round(fabs(repeats)*9 + 1);

  T lineVector[2][1];
  m_minus<T, 2, 1>(lineVector, localPrev, localTrgt);

  T normedLineVector[2][1];
  m_copy<T, 2, 1>(normedLineVector, lineVector);
  m_norm<T, 2>(normedLineVector);

  if (partNumber > 0 && partNumber < partsCount) {
    /* maneuver parts */
    T northOffset = height/2.0 - radius;
    T eastOffset = width/2.0 - radius;
    T semiWidth = width/2.0;
    T semiHeight = height/2.0;

    switch (partNumber % 9) {
      case 1:
        part.fillLine(0.0, -semiWidth, northOffset, -semiWidth);
        part.setFinal(false);
        break;

      case 2:
        part.fillArc(northOffset, -eastOffset, radius, 0.0, M_PI_2);
        part.setFinal(false);
        break;

      case 3:
        part.fillLine(semiHeight, -eastOffset, semiHeight,  eastOffset);
        part.setFinal(false);
        break;

      case 4:
        part.fillArc(northOffset, eastOffset, radius, M_PI_2, M_PI_2);
        part.setFinal(false);
        break;

      case 5:
        part.fillLine(northOffset, semiWidth, -northOffset, semiWidth);
        part.setFinal(false);
        break;

      case 6:
        part.fillArc(-northOffset, eastOffset, radius, M_PI, M_PI_2);
        part.setFinal(false);
        break;

      case 7:
        part.fillLine(-semiHeight,  eastOffset, -semiHeight, -eastOffset);
        part.setFinal(false);
        break;

      case 8:
        part.fillArc(-northOffset, -eastOffset, radius, 3.0*M_PI_2, M_PI_2);
        part.setFinal(false);
        break;

      case 0:
        part.fillLine(-northOffset, -semiWidth, 0.0, -semiWidth);
        part.setFinal(false);
        break;
    }

    if (sign(repeats) < 0.0)
      part.flipEast();

    part.rotate(deg2rad<T>(angle));
    part.move(localTrgt);

  }
  else if (0 == partNumber) {
    /* line from previous waypoint to the stadium's border */
    part.fillLine(0.0, 0.0, 0.0, -width/2.0);
    part.setFinal(false);
    part.rotate(deg2rad<T>(angle));
    part.move(localTrgt);
    ManeuverLine<T> line = part.getLine();
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
  else {
    /* out of maneuver parts range */
    part.fillUnknown();

  }

}

} /* namespace maneuver */
} /* namespace control */


#endif /* MANEUVER_STADIUM_HPP_ */
