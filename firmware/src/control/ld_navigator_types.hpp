#ifndef LD_NAVIGATOR_TYPES_HPP_
#define LD_NAVIGATOR_TYPES_HPP_

namespace control {

enum class ManeuverPartType {
  line,
  arc,
  unknown
};

template <typename T>
struct LdNavOut {
  LdNavOut(void) : dz(0), dist(0), crs(0) , crossed(false) {;}
  LdNavOut(T dz, T dist, T crs, bool crossed) :
    dz(dz), dist(dist), crs(crs), crossed(crossed) {;}
  T dz;         /* cross track error (m) */
  T dist;       /* distance to target point (m) */
  T crs;        /* course to target point (rad), [0; 2*M_PI] */
  bool crossed; /* is mission part crossed  */
};

template <typename T>
struct ManeuverLine {
  T start[2][1];  /* north (m), east (m) coordinates */
  T finish[2][1]; /* north (m), east (m) coordinates */
};

template <typename T>
struct ManeuverArc {
  T center[2][1]; /* north (m), east (m) coordinates */
  T radius;       /* arc radius (m), if positive - clockwise, else counter-clockwise */
  T startCourse;  /* course (rad) on arc start, [0, 2*M_PI] */
  T deltaCourse;  /* course change (rad) on arc, [0; M_PI] */
};

/**
 * @brief Mission part: line or arc.
 */
template <typename T>
struct ManeuverPart {
  ManeuverPart(void) {
    memset(this, 0, sizeof(ManeuverPart));
    finale = true;
    type = ManeuverPartType::unknown;
  }
  void fillLine(T (&start)[2][1], T (&finish)[2][1], bool finale) {
    type = ManeuverPartType::line;
    this->finale = finale;
    m_copy<T, 2, 1>(line.start, start);
    m_copy<T, 2, 1>(line.finish, finish);
  }
  void fillLine(T startNorth, T startEast,
                T finishNorth, T finishEast, bool finale) {
    type = ManeuverPartType::line;
    this->finale = finale;
    line.start[0][0] = startNorth;
    line.start[1][0] = startEast;
    line.finish[0][0] = finishNorth;
    line.finish[1][0] = finishEast;
  }
  void fillArc(T (&center)[2][1], T radius,
               T startCourse, T deltaCourse, bool finale) {
    type = ManeuverPartType::arc;
    this->finale = finale;
    m_copy<T, 2, 1>(arc.center, center);
    arc.radius = radius;
    arc.startCourse = startCourse;
    arc.deltaCourse = deltaCourse;
  }
  void fillArc(T centerNorth, T centerEast, T radius,
               T startCourse, T deltaCourse, bool finale) {
    type = ManeuverPartType::arc;
    this->finale = finale;
    arc.center[0][0] = centerNorth;
    arc.center[1][0] = centerEast;
    arc.radius = radius;
    arc.startCourse = startCourse;
    arc.deltaCourse = deltaCourse;
  }
  void fillUnknown() {
    type = ManeuverPartType::unknown;
    finale = true;
  }
  bool finale;            /* is last part in maneuver */
  ManeuverPartType type;  /* type of maneuver part */
  union {
    ManeuverArc<T> arc;
    ManeuverLine<T> line;
  };
};

} /* control namespace */

#endif /* LD_NAVIGATOR_TYPES_HPP_ */
