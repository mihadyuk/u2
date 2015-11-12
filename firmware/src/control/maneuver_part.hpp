#ifndef MANEUVER_PART_HPP_
#define MANEUVER_PART_HPP_

#include <math.h>

#include "ld_navigator_types.hpp"
#include "matrix_math.hpp"

namespace control {

template <typename T>
class ManeuverPart {
public:
  ManeuverPart();
  void fillLine(T (&start)[2][1], T (&finish)[2][1], bool finale);
  void fillLine(T startNorth, T startEast,
                T finishNorth, T finishEast, bool finale);
  void fillArc(T (&center)[2][1], T radius,
               T startCourse, T deltaCourse, bool finale);
  void fillArc(T centerNorth, T centerEast, T radius,
               T startCourse, T deltaCourse, bool finale);
  void fillUnknown();
  void rotate(T angle);
  void flipNorth();
  void flipEast();
  void move(T (&delta)[2][1]);

  ManeuverPartType type; /* type of maneuver part */
  bool finale;           /* is last part in maneuver */
  ManeuverArc<T> arc;
  ManeuverLine<T> line;

};

template <typename T>
ManeuverPart<T>::ManeuverPart() {
  memset(this, 0, sizeof(ManeuverPart<T>));
  finale = true;
  type = ManeuverPartType::unknown;
}

template <typename T>
void ManeuverPart<T>::fillLine(T (&start)[2][1],
                               T (&finish)[2][1],
                               bool finale) {
  line.fill(start, finish);
  arc.clear();
  type = ManeuverPartType::line;
  this->finale = finale;
}

template <typename T>
void ManeuverPart<T>::fillLine(T startNorth, T startEast,
                               T finishNorth, T finishEast,
                               bool finale) {
  line.fill(startNorth, startEast, finishNorth, finishEast);
  arc.clear();
  type = ManeuverPartType::line;
  this->finale = finale;
}

template <typename T>
void ManeuverPart<T>::fillArc(T (&center)[2][1], T radius,
                              T startCourse, T deltaCourse,
                              bool finale) {
  arc.fill(center, radius, startCourse, deltaCourse);
  line.clear();
  type = ManeuverPartType::arc;
  this->finale = finale;
}

template <typename T>
void ManeuverPart<T>::fillArc(T centerNorth, T centerEast,
                              T radius, T startCourse, T deltaCourse,
                              bool finale) {
  arc.fill(centerNorth, centerEast, radius, startCourse, deltaCourse);
  line.clear();
  type = ManeuverPartType::arc;
  this->finale = finale;
}

template <typename T>
void ManeuverPart<T>::fillUnknown() {
  line.clear();
  arc.clear();
  type = ManeuverPartType::unknown;
  finale = true;
}

template <typename T>
void ManeuverPart<T>::rotate(T angle) {
  T sinAng = sin(angle);
  T cosAng = cos(angle);
  T dcm[2][2] = {{cosAng, -sinAng},
                 {sinAng,  cosAng}};
  T tmp[2][1];

  switch (type) {
    case ManeuverPartType::line: {
      m_mul<T, 2, 2, 1>(tmp, dcm, line.start);
      m_copy<T, 2, 1>(line.start, tmp);
      m_mul<T, 2, 2, 1>(tmp, dcm, line.finish);
      m_copy<T, 2, 1>(line.finish, tmp);
      break;
    }
    case ManeuverPartType::arc: {
      m_mul<T, 2, 2, 1>(tmp, dcm, arc.center);
      m_copy<T, 2, 1>(arc.center, tmp);
      arc.startCourse = wrap_2pi(arc.startCourse + angle);
      break;
    }
    default:
      break;
  }
}

template <typename T>
void ManeuverPart<T>::flipNorth() {
  switch (type) {
    case ManeuverPartType::line: {
      line.start[1][0] = -line.start[1][0];
      line.finish[1][0] = -line.finish[1][0];
      break;
    }
    case ManeuverPartType::arc: {
      arc.center[1][0] = -arc.center[1][0];
      arc.radius = -arc.radius;
      arc.startCourse = wrap_2pi(static_cast<T>(2.0*M_PI) - arc.startCourse);
      break;
    }
    default:
      break;
  }
}

template <typename T>
void ManeuverPart<T>::flipEast() {
  switch (type) {
    case ManeuverPartType::line: {
      line.start[0][0] = -line.start[0][0];
      line.finish[0][0] = -line.finish[0][0];
      break;
    }
    case ManeuverPartType::arc: {
      arc.center[0][0] = -arc.center[0][0];
      arc.radius = -arc.radius;
      arc.startCourse = wrap_2pi(static_cast<T>(M_PI) - arc.startCourse);
      break;
    }
    default:
      break;
  }
}

template <typename T>
void ManeuverPart<T>::move(T (&delta)[2][1]) {
  switch (type) {
    case ManeuverPartType::arc: {
      m_plus<T, 2, 1>(arc.center, arc.center, delta);
      break;
    }
    case ManeuverPartType::line: {
      m_plus<T, 2, 1>(line.start, line.start, delta);
      m_plus<T, 2, 1>(line.finish, line.finish, delta);
      break;
    }
    default:
      break;
  }
}

} /* namespace control */

#endif /* MANEUVER_PART_HPP_ */
