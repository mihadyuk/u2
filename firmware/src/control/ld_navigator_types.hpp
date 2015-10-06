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
  T crs;        /* course to target point (rad) */
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
  T radius;       /* arc radius (m) */
  T startCourse;  /* course (rad) on arc start */
  T deltaCourse;  /* course change (rad) on arc */
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
  bool finale;            /* is last part in maneuver */
  ManeuverPartType type;  /* type of maneuver part */
  union {
    ManeuverArc<T> arc;
    ManeuverLine<T> line;
  };
};

} /* control namespace */

#endif /* LD_NAVIGATOR_TYPES_HPP_ */
