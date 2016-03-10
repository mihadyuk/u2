#ifndef MANEUVER_THREE_POINTS_HPP_
#define MANEUVER_THREE_POINTS_HPP_

#include <math.h>
#include "maneuver_part.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"

namespace control {
namespace maneuver {

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
      part.fillLine(localPrev, localTrgt);
      part.setFinal(true);
      return;
    }

    T lineStart[2][1];
    m_copy<T, 2, 1>(lineStart, trgtToPrevVect);
    m_norm<T, 2>(trgtToPrevVect);

    T alpha = deltaCrs/2;

    if (deltaCrs >= static_cast<T>(0.0)) {
      radius = -fabs(radius);
    }
    else {
      radius = fabs(radius);
    }

    switch (partNumber) {
      case 0: {
        T lineFinish[2][1];
        T arm = static_cast<T>(-radius)/tan(alpha);
        // Check if arc's arm more than distance between waypoints
        if (arm > distTrgtToPrev || arm > distTrgtToThird) {
          part.fillLine(localPrev, localTrgt);
          part.setFinal(true);
          return;
        }
        m_mul_s<T, 2, 1>(lineFinish, trgtToPrevVect, arm);
        part.fillLine(lineStart, lineFinish);
        part.setFinal(false);
      }
        break;

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

        part.fillArc(arcCenter, radius, startCrs, dCrs);
        part.setFinal(true);
      }
        break;
    }

    part.move(localTrgt);

  }
  else {
    /* out of maneuver parts range */
    part.fillUnknown();
  }

}

} /* namespace maneuver */
} /* namespace control */

#endif /* MANEUVER_THREE_POINTS_HPP_ */
