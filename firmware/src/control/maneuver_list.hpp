#ifndef MANEUVER_LIST_HPP_
#define MANEUVER_LIST_HPP_

#include <math.h>
#include "ld_navigator_types.hpp"
#include "matrix_math.hpp"
#include "e_frame.hpp"
#include "geometry.hpp"
#include "pads.h"

namespace control {
namespace maneuver {

template <typename T>
void missionItemWGS84toNE(T (&localNE)[2][1],
                          T (&currWGS84)[3][1],
                          const mavlink_mission_item_t &wp) {
  T wpWGS84[3][1] = {{deg2rad<T>(wp.x)},
                     {deg2rad<T>(wp.y)},
                     {deg2rad<T>(wp.z)}};
  T wpLocalNED[3][1];
  geo2lv(wpLocalNED, wpWGS84, currWGS84);
  get_sub<T, 2, 1, 3, 1>(localNE, wpLocalNED, 0, 0);
}

template <typename T>
void updateMnrLine(ManeuverPart<T> &part,
                   T (&prevNE)[2][1],
                   T (&trgtNE)[2][1]) {

  part.type = ManeuverPartType::line;
  part.finale = true;
  m_copy<T, 2, 1>(part.line.start, prevNE);
  m_copy<T, 2, 1>(part.line.finish, trgtNE);
}

template <typename T>
void updateMnrCircle(ManeuverPart<T> &part,
                     size_t partNumber,
                     T turns,
                     T radius,
                     T (&prevNE)[2][1],
                     T (&trgtNE)[2][1]) {
  size_t partsCount = static_cast<size_t>(round(turns))*2 + 2;

  T lineVector[2][1];
  m_minus<T, 2, 1>(lineVector, prevNE, trgtNE);

  T normedLineVector[2][1];
  m_copy<T, 2, 1>(normedLineVector, lineVector);
  m_norm<T, 2>(normedLineVector);

  if (partNumber > 0 && partNumber < (partsCount - 1)) {
    /* semicircles */
    T cwSign = sign<T>(radius);
    T lineCourse = atan2(normedLineVector[1][0],
                         normedLineVector[0][0])
                 + cwSign*static_cast<T>(M_PI_2);
    lineCourse = wrap_2pi(lineCourse);

    part.type = ManeuverPartType::arc;
    part.finale = false;
    part.arc.radius = radius;
    m_copy<T, 2, 1>(part.arc.center, trgtNE);
    part.arc.deltaCourse = M_PI;

    size_t semiCircleNumber = partNumber % 2;
    if (1 == semiCircleNumber) {
      part.arc.startCourse = lineCourse;

      red_led_toggle();
    } else {
      part.arc.startCourse = wrap_2pi(lineCourse + static_cast<T>(M_PI));

      green_led_toggle();
    }

  } else if (0 == partNumber) {
    /* line from previous waypoint to the circle's border */
    part.type = ManeuverPartType::line;
    part.finale = false;
    m_copy<T, 2, 1>(part.line.start, prevNE);
    m_mul_s<T, 2, 1>(normedLineVector, normedLineVector, fabs(radius));
    m_plus<T, 2, 1>(part.line.finish, trgtNE, normedLineVector);

    blue_led_toggle();

  } else if ((partsCount - 1) == partNumber) {
    /* line from the circle's border to the circle's center */
    part.type = ManeuverPartType::line;
    part.finale = true;
    m_copy<T, 2, 1>(part.line.finish, trgtNE);
    m_mul_s<T, 2, 1>(normedLineVector, normedLineVector, fabs(radius));
    m_plus<T, 2, 1>(part.line.start, trgtNE, normedLineVector);

    blue_led_toggle();

  } else {
    part.type = ManeuverPartType::unknown;
    part.finale = true;
  }

}

template <typename T>
void updateMnrThreePoints(ManeuverPart<T> &part,
                          size_t partNumber,
                          T radius,
                          T (&prevNE)[2][1],
                          T (&trgtNE)[2][1],
                          T (&thirdNE)[2][1]) {
  //TODO
}

template <typename T>
void updateMnrUnknown(ManeuverPart<T> &part) {
  part.type = ManeuverPartType::unknown;
  part.finale = true;
}

} /* namespace maneuver */
} /* namespace control */

#endif /* MANEUVER_LIST_HPP_ */
