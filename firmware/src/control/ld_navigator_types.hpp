#ifndef LD_NAVIGATOR_TYPES_HPP_
#define LD_NAVIGATOR_TYPES_HPP_

#include <string.h>
#include "matrix_math.hpp"

namespace control {

enum class ManeuverPartType {
  line,
  arc,
  unknown
};

template <typename T>
struct LdNavOut {
  LdNavOut(void) : dz(0), dist(0), crs(0) , crossed(true) {;}
  LdNavOut(T dz, T dist, T crs, bool crossed) :
    dz(dz), dist(dist), crs(crs), crossed(crossed) {;}
  T dz;         /* cross track error (m) */
  T dist;       /* distance to target point (m) */
  T crs;        /* course to target point (rad), [0; 2*M_PI] */
  bool crossed; /* is mission part crossed  */
};

template <typename T>
struct ManeuverLine {
  T start[2][1];  /* local north (m), east (m) coordinates */
  T finish[2][1]; /* local north (m), east (m) coordinates */

  void fill(const T (&start)[2][1], const T (&finish)[2][1]) {
    m_copy<T, 2, 1>(this->start, start);
    m_copy<T, 2, 1>(this->finish, finish);
  }
  void fill(T startNorth, T startEast,
            T finishNorth, T finishEast) {
    this->start[0][0] = startNorth;
    this->start[1][0] = startEast;
    this->finish[0][0] = finishNorth;
    this->finish[1][0] = finishEast;
  }
  void clear() {
    memset(this, 0, sizeof(ManeuverLine<T>));
  }
};

template <typename T>
struct ManeuverArc {
  T center[2][1]; /* local north (m), east (m) coordinates */
  T radius;       /* arc radius (m), if positive - clockwise, else counter-clockwise */
  T startCourse;  /* course (rad) on arc start, [0, 2*M_PI] */
  T deltaCourse;  /* course change (rad) on arc, [0; M_PI] */

  void fill(const T (&center)[2][1], T radius,
            T startCourse, T deltaCourse) {
    m_copy<T, 2, 1>(this->center, center);
    this->radius = radius;
    this->startCourse = startCourse;
    this->deltaCourse = deltaCourse;
  }
  void fill(T centerNorth, T centerEast, T radius,
            T startCourse, T deltaCourse) {
    this->center[0][0] = centerNorth;
    this->center[1][0] = centerEast;
    this->radius = radius;
    this->startCourse = startCourse;
    this->deltaCourse = deltaCourse;
  }
  void clear() {
    memset(this, 0, sizeof(ManeuverArc<T>));
  }
};

/**
 * @brief Maneuver part: line or arc.
 */
//template <typename T>
//struct MnrPart {
//  MnrPart(void) {
//    memset(this, 0, sizeof(MnrPart<T>));
//    finale = true;
//    type = ManeuverPartType::unknown;
//  }
//  void fillLine(const T (&start)[2][1], const T (&finish)[2][1], bool finale) {
//    type = ManeuverPartType::line;
//    this->finale = finale;
//    m_copy<T, 2, 1>(line.start, start);
//    m_copy<T, 2, 1>(line.finish, finish);
//  }
//  void fillLine(T startNorth, T startEast,
//                T finishNorth, T finishEast, bool finale) {
//    type = ManeuverPartType::line;
//    this->finale = finale;
//    line.start[0][0] = startNorth;
//    line.start[1][0] = startEast;
//    line.finish[0][0] = finishNorth;
//    line.finish[1][0] = finishEast;
//  }
//  void fillArc(const T (&center)[2][1], T radius,
//               T startCourse, T deltaCourse, bool finale) {
//    type = ManeuverPartType::arc;
//    this->finale = finale;
//    m_copy<T, 2, 1>(arc.center, center);
//    arc.radius = radius;
//    arc.startCourse = startCourse;
//    arc.deltaCourse = deltaCourse;
//  }
//  void fillArc(T centerNorth, T centerEast, T radius,
//               T startCourse, T deltaCourse, bool finale) {
//    type = ManeuverPartType::arc;
//    this->finale = finale;
//    arc.center[0][0] = centerNorth;
//    arc.center[1][0] = centerEast;
//    arc.radius = radius;
//    arc.startCourse = startCourse;
//    arc.deltaCourse = deltaCourse;
//  }
//  void fillUnknown() {
//    type = ManeuverPartType::unknown;
//    finale = true;
//  }
//  bool finale;            /* is last part in maneuver */
//  ManeuverPartType type;  /* type of maneuver part */
//  union {
//    ManeuverArc<T> arc;
//    ManeuverLine<T> line;
//  };
//};

} /* control namespace */

#endif /* LD_NAVIGATOR_TYPES_HPP_ */
