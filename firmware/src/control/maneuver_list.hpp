#ifndef MANEUVER_LIST_HPP_
#define MANEUVER_LIST_HPP_

#include <math.h>
#include "ld_navigator_types.hpp"
#include "maneuver_part.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"

namespace control {

template <typename T>
void circleManeuver(ManeuverPart<T> &part,
                    uint32_t partNumber,
                    float repeats, float radius,
                    const T (&localPrev)[2][1],
                    const T (&localTrgt)[2][1]) {

  uint32_t partsCount = round(fabs(repeats)*4 + 1);

  T lineVector[2][1];
  m_minus<T, 2, 1>(lineVector, localPrev, localTrgt);

  T normedLineVector[2][1];
  m_copy<T, 2, 1>(normedLineVector, lineVector);
  m_norm<T, 2>(normedLineVector);

  if (partNumber > 0 && partNumber < partsCount) {
    // quarter circle
    T cwSign = sign<T>(radius);
    T tangentCourse = tangentLineCourse(normedLineVector, cwSign);

    part.fillArc(localTrgt, radius, 0.0, M_PI_2, false);

    switch (partNumber % 4) {
      case 1:
        part.arc.startCourse = tangentCourse;
        break;
      case 2:
        part.arc.startCourse = wrap_2pi(tangentCourse +
                                        cwSign*static_cast<T>(M_PI_2));
        break;
      case 3:
        part.arc.startCourse = wrap_2pi(tangentCourse +
                                        cwSign*static_cast<T>(M_PI));
        break;
      case 0:
        part.arc.startCourse = wrap_2pi(tangentCourse +
                                        cwSign*static_cast<T>(3.0*M_PI_2));
        break;
      default:
        break;
    }

    if ((partsCount - 1) == partNumber)
          part.finale = true;

  } else if (0 == partNumber) {
    // line from previous waypoint to the circle's border
    part.fillLine(localPrev, localTrgt, false);
    m_mul_s<T, 2, 1>(normedLineVector,
                     normedLineVector,
                     fabs(radius));
    m_plus<T, 2, 1>(part.line.finish,
                    localTrgt,
                    normedLineVector);
  } else {
    part.fillUnknown();
  }

}

template <typename T>
void threePointsManeuver(ManeuverPart<T> &part,
                         uint32_t partNumber,
                         float radius,
                         const T (&localPrev)[2][1],
                         const T (&localTrgt)[2][1],
                         const T (&localThird)[2][1]) {
  if (partNumber < 2) {

    T trgtToPrevVect[2][1];
    m_minus<T, 2, 1>(trgtToPrevVect, localPrev, localTrgt);
    T distTrgtToPrev = m_vec_norm<T, 2>(trgtToPrevVect);
    T trgtToThirdVect[2][1];
    m_minus<T, 2, 1>(trgtToThirdVect, localThird, localTrgt);
    T distTrgtToThird = m_vec_norm<T, 2>(trgtToThirdVect);

    T trgtToPrevCrs = atan2(trgtToPrevVect[1][0],
                            trgtToPrevVect[0][0]);
    T trgtToThirdCrs = atan2(trgtToThirdVect[1][0],
                             trgtToThirdVect[0][0]);

    T deltaCrs = wrap_pi(trgtToThirdCrs - trgtToPrevCrs);
    // Check if previous, target and third waypoints are on the one line
    if (static_cast<T>(0.0) == deltaCrs) {
      part.fillLine(localPrev, localTrgt, true);
      return;
    }

    T lineStart[2][1];
    m_copy<T, 2, 1>(lineStart, trgtToPrevVect);
    m_norm<T, 2>(trgtToPrevVect);

    T alpha = deltaCrs/2;

    if (deltaCrs >= static_cast<T>(0.0))
      radius = -fabs(radius);
    else
      radius = fabs(radius);

    switch (partNumber) {
      case 0: {
        T lineFinish[2][1];
        T arm = static_cast<T>(-radius)/tan(alpha);
        // Check if arc's arm more than distance between waypoints
        if (arm > distTrgtToPrev ||
            arm > distTrgtToThird) {
          part.fillLine(localPrev, localTrgt, true);
          return;
        }

        m_mul_s<T, 2, 1>(lineFinish,
                         trgtToPrevVect,
                         arm);

        part.fillLine(lineStart, lineFinish, false);
        break;
      }
      case 1: {
        T cosAlpha = cos(alpha);
        T sinAlpha = sin(alpha);
        T C[2][2] = {{cosAlpha, -sinAlpha},
                     {sinAlpha,  cosAlpha}};

        T trgtToArcCenter[2][1];
        m_mul<T, 2, 2, 1>(trgtToArcCenter, C, trgtToPrevVect);
        T arcCenter[2][1];
        m_mul_s<T, 2, 1>(arcCenter,
                         trgtToArcCenter,
                         static_cast<T>(-radius)/sinAlpha);

        T startCrs = atan2(-trgtToPrevVect[1][0],
                           -trgtToPrevVect[0][0]);
        startCrs = wrap_2pi(static_cast<T>(sign(radius))*startCrs);
        T dCrs = static_cast<T>(2.0)*(static_cast<T>(M_PI_2) - fabs(alpha));
        dCrs = wrap_2pi(dCrs);

        part.fillArc(arcCenter, radius, startCrs, dCrs, true);
        break;
      }
      default:
        break;
    }

    part.move(localTrgt);

  } else {
    part.fillUnknown();
  }

}

template <typename T>
void infinityManeuver(ManeuverPart<T> &part,
                      uint32_t partNumber,
                      float repeats, float radius, float height, float angle,
                      const T (&localPrev)[2][1],
                      const T (&localTrgt)[2][1]) {

  uint32_t partsCount = round(fabs(repeats)*7 + 1);

  if (0 == partNumber) {
    /* line from previous waypoint to the infinity's center */
    part.fillLine(localPrev, localTrgt, false);

  } else if (partNumber > (partsCount - 1)) {
    /* out of maneuver parts range */
    part.fillUnknown();

  } else {
    /* maneuver parts */
    T distToArcCenter = height/2.0 - radius;
    T arm = sqrt(distToArcCenter*distToArcCenter -
                 static_cast<T>(radius*radius));
    T alpha = asin(static_cast<T>(radius)/distToArcCenter);

    switch (partNumber % 7) {
      case 1:
        part.fillLine(0.0, 0.0, arm, 0.0, false);
        part.rotate(alpha);
        break;
      case 2:
        part.fillArc(distToArcCenter, 0.0,
                     -fabs(radius),
                     wrap_2pi(alpha),
                     static_cast<T>(M_PI_2) + alpha,
                     false);
        break;
      case 3:
        part.fillArc(distToArcCenter, 0.0,
                     -fabs(radius),
                     3.0*M_PI_2,
                     static_cast<T>(M_PI_2) + alpha,
                     false);
        break;
      case 4:
        part.fillLine(arm, 0.0, -arm, 0.0, false);
        part.rotate(-alpha);
        break;
      case 5:
        part.fillArc(-distToArcCenter, 0.0,
                     fabs(radius),
                     wrap_2pi(-alpha + static_cast<T>(M_PI)),
                     static_cast<T>(M_PI_2) + alpha,
                     false);
        break;
      case 6:
        part.fillArc(-distToArcCenter, 0.0,
                     fabs(radius),
                     3.0*M_PI_2,
                     static_cast<T>(M_PI_2) + alpha,
                     false);
        break;
      case 0:
        part.fillLine(-arm, 0.0, 0.0, 0.0, false);
        part.rotate(alpha);
        break;
      default:
        break;
    }

    if (sign(repeats) < 0.0)
      part.flipNorth();

    part.rotate(deg2rad<T>(angle));
    part.move(localTrgt);

  }

  if ((partsCount - 1) == partNumber)
    part.finale = true;

}

template <typename T>
void stadiumManeuver(ManeuverPart<T> &part,
                     uint32_t partNumber,
                     float repeats, float radius, float height, float width, float angle,
                     const T (&localPrev)[2][1],
                     const T (&localTrgt)[2][1]) {

  uint32_t partsCount = round(fabs(repeats)*9 + 2);

  T lineVector[2][1];
  m_minus<T, 2, 1>(lineVector, localPrev, localTrgt);

  T normedLineVector[2][1];
  m_copy<T, 2, 1>(normedLineVector, lineVector);
  m_norm<T, 2>(normedLineVector);

  if (partNumber > 0 &&
      partNumber < (partsCount - 1)) {
    /* maneuver parts */
    T northOffset = height/2.0 - radius;
    T eastOffset = width/2.0 - radius;
    T semiWidth = width/2.0;
    T semiHeight = height/2.0;

    switch (partNumber % 9) {
      case 1:
        part.fillLine(0.0,         -semiWidth,
                      northOffset, -semiWidth,
                      false);
        break;
      case 2:
        part.fillArc(northOffset, -eastOffset,
                     radius,
                     0.0,          M_PI_2,
                     false);
        break;
      case 3:
        part.fillLine(semiHeight, -eastOffset,
                      semiHeight,  eastOffset,
                      false);
        break;
      case 4:
        part.fillArc(northOffset, eastOffset,
                     radius,
                     M_PI_2,      M_PI_2,
                     false);
        break;
      case 5:
        part.fillLine(northOffset, semiWidth,
                     -northOffset, semiWidth,
                      false);
        break;
      case 6:
        part.fillArc(-northOffset, eastOffset,
                     radius,
                     M_PI,         M_PI_2,
                     false);
        break;
      case 7:
        part.fillLine(-semiHeight,  eastOffset,
                      -semiHeight, -eastOffset,
                      false);
        break;
      case 8:
        part.fillArc(-northOffset, -eastOffset,
                     radius,
                     3.0*M_PI_2,    M_PI_2,
                     false);
        break;
      case 0:
        part.fillLine(-northOffset, -semiWidth,
                       0.0,         -semiWidth,
                      false);
        break;
      default:
        break;
    }

    if (sign(repeats) < 0.0)
      part.flipEast();

    part.rotate(deg2rad<T>(angle));
    part.move(localTrgt);

  } else if (0 == partNumber) {
    /* line from previous waypoint to the stadium's border */
    part.fillLine(0.0, 0.0, 0.0, -width/2.0, false);
    part.rotate(deg2rad<T>(angle));
    part.move(localTrgt);
    m_copy<T, 2, 1>(part.line.start, localPrev);

  } else if ((partsCount - 1) == partNumber) {
    /* line from the stadium's border to the stadium's center */
    part.fillLine(0.0, -width/2.0, 0.0, 0.0, false);
    part.rotate(deg2rad<T>(angle));
    part.move(localTrgt);

  } else {
    part.fillUnknown();

  }

}

} /* namespace control */

#endif /* MANEUVER_LIST_HPP_ */
