#ifndef MANEUVER_CIRCLE_HPP_
#define MANEUVER_CIRCLE_HPP_

#include <math.h>
#include "maneuver_part.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"

namespace control {
namespace maneuver {

template <typename T>
void circleManeuver(ManeuverPart<T> &part,
                    uint32_t partNumber,
                    float repeats,
                    float radius,
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
    T startCourse = 0.0;

    switch (partNumber % 4) {
      case 1:
        startCourse = tangentCourse;
        break;

      case 2:
        startCourse = wrap_2pi(tangentCourse +
                               cwSign*static_cast<T>(M_PI_2));
        break;

      case 3:
        startCourse = wrap_2pi(tangentCourse +
                               cwSign*static_cast<T>(M_PI));
        break;

      case 0:
        startCourse = wrap_2pi(tangentCourse +
                               cwSign*static_cast<T>(3.0*M_PI_2));
        break;
    }

    part.fillArc(localTrgt, radius, startCourse, M_PI_2);

    if ((partsCount - 1) == partNumber) {
      part.setFinal(true);
    }
    else {
      part.setFinal(false);
    }

  }
  else if (0 == partNumber) {
    // line from previous waypoint to the circle's border
    T tmp[2][1];
    m_mul_s<T, 2, 1>(tmp, normedLineVector, fabs(radius));
    m_plus<T, 2, 1>(tmp, localTrgt, tmp);
    part.fillLine(localPrev, tmp);
    part.setFinal(false);

  }
  else {
    /* out of maneuver parts range */
    part.fillUnknown();
  }

}

} /* namespace maneuver */
} /* namespace control */

#endif /* MANEUVER_CIRCLE_HPP_ */
