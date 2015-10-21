#ifndef MANEUVER_LIST_HPP_
#define MANEUVER_LIST_HPP_

#include <math.h>
#include "ld_navigator_types.hpp"
#include "matrix_math.hpp"
#include "e_frame.hpp"
#include "geometry.hpp"

namespace control {
namespace maneuver {

template <typename T>
void lineMnr(ManeuverPart<T> &part,
             T (&prevNE)[2][1],
             T (&trgtNE)[2][1]) {

  part.type = ManeuverPartType::line;
  part.finale = true;
  m_copy<T, 2, 1>(part.line.start, prevNE);
  m_copy<T, 2, 1>(part.line.finish, trgtNE);
}

template <typename T>
void circleMnr(ManeuverPart<T> &part,
               uint32_t partNumber,
               T repeats,
               T radius,
               T (&prevNE)[2][1],
               T (&trgtNE)[2][1]) {
  uint32_t partsCount = static_cast<uint32_t>(round(fabs(repeats)))*2 + 2;

  T lineVector[2][1];
  m_minus<T, 2, 1>(lineVector, prevNE, trgtNE);

  T normedLineVector[2][1];
  m_copy<T, 2, 1>(normedLineVector, lineVector);
  m_norm<T, 2>(normedLineVector);

  if (partNumber > 0 && partNumber < (partsCount - 1)) {
    /* semicircles */
    T cwSign = sign<T>(radius);
    T lineCourse = atan2(normedLineVector[1][0],
                         normedLineVector[0][0]) +
                             cwSign*static_cast<T>(M_PI_2);
    lineCourse = wrap_2pi(lineCourse);

    part.type = ManeuverPartType::arc;
    part.finale = false;
    part.arc.radius = radius;
    m_copy<T, 2, 1>(part.arc.center, trgtNE);
    part.arc.deltaCourse = M_PI;

    size_t semiCircleNumber = partNumber % 2;
    if (1 == semiCircleNumber) {
      part.arc.startCourse = lineCourse;
    } else {
      part.arc.startCourse = wrap_2pi(lineCourse +
                                      static_cast<T>(M_PI));
    }

  } else if (0 == partNumber) {
    /* line from previous waypoint to the circle's border */
    part.type = ManeuverPartType::line;
    part.finale = false;
    m_copy<T, 2, 1>(part.line.start, prevNE);
    m_mul_s<T, 2, 1>(normedLineVector, normedLineVector, fabs(radius));
    m_plus<T, 2, 1>(part.line.finish, trgtNE, normedLineVector);

  } else if ((partsCount - 1) == partNumber) {
    /* line from the circle's border to the circle's center */
    part.type = ManeuverPartType::line;
    part.finale = true;
    m_copy<T, 2, 1>(part.line.finish, trgtNE);
    m_mul_s<T, 2, 1>(normedLineVector, normedLineVector, fabs(radius));
    m_plus<T, 2, 1>(part.line.start, trgtNE, normedLineVector);

  } else {
    part.type = ManeuverPartType::unknown;
    part.finale = true;

  }

}

template <typename T>
void unknownMnr(ManeuverPart<T> &part) {
  part.type = ManeuverPartType::unknown;
  part.finale = true;
}

} /* namespace maneuver */
} /* namespace control */

#endif /* MANEUVER_LIST_HPP_ */
