#ifndef CONTROL_NAVIGATOR_TYPES_HPP_
#define CONTROL_NAVIGATOR_TYPES_HPP_

namespace control {

/**
 * @note   All values in radians.
 */
template <typename T>
struct NavIn {
  NavIn(void) : lat(0), lon(0) {;}
  NavIn(T lat, T lon) : lat(lat), lon(lon) {;}
  T lat;
  T lon;
};

/**
 * @note   All values in radians.
 */
template <typename T>
struct NavOut {
  NavOut(void) : xtd(0), atd(0), dist(0), crs(0) {;}
  NavOut(T xtd, T atd, T dist, T crs) : xtd(xtd), atd(atd), dist(dist), crs(crs) {;}
  T xtd;  /* cross track error */
  T atd;  /* along track distance */
  T dist; /* distance to target point */
  T crs;  /* course to target point */
};

/**
 * @brief   Mission line from A to B.
 * @note    All values in radians.
 */
template <typename T>
struct NavLine {
  NavLine(void) : latA(0), lonA(0), latB(0), lonB(0) {;}
  NavLine(T latA, T lonA, T latB, T lonB) : latA(latA), lonA(lonA), latB(latB), lonB(lonB) {;}
  T latA;
  T lonA;
  T latB;
  T lonB;
};

} /* namespace */

#endif /* CONTROL_NAVIGATOR_TYPES_HPP_ */
