#ifndef CONTROL_NAVIGATOR_TYPES_HPP_
#define CONTROL_NAVIGATOR_TYPES_HPP_

namespace control {

/**
 *
 */
template <typename T>
struct NavIn {
  NavIn(void) : lat(0), lon(0) {;}
  NavIn(T lat, T lon) : lat(lat), lon(lon) {;}
  T lat; /* (rad) */
  T lon; /* (rad) */
};

/**
 *
 */
template <typename T>
struct NavOut {
  NavOut(void) : xtd(0), atd(0), dist(0), crs(0) {;}
  NavOut(T xtd, T atd, T dist, T crs) : xtd(xtd), atd(atd), dist(dist), crs(crs) {;}
  T xtd;  /* cross track error (rad) */
  T atd;  /* along track distance (rad) */
  T dist; /* distance to target point (rad) */
  T crs;  /* course to target point (rad) */
};

/**
 * @brief   Mission line from A to B.
 */
template <typename T>
struct NavLine {
  NavLine(void) : latA(0), lonA(0), latB(0), lonB(0) {;}
  NavLine(T latA, T lonA, T latB, T lonB) : latA(latA), lonA(lonA), latB(latB), lonB(lonB) {;}
  T latA; /* (rad) */
  T lonA; /* (rad) */
  T latB; /* (rad) */
  T lonB; /* (rad) */
};

} /* namespace */

#endif /* CONTROL_NAVIGATOR_TYPES_HPP_ */
