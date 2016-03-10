#ifndef GEOMETRY_HPP_
#define GEOMETRY_HPP_

#include <cmath>

#define DEG_TO_RAD  0.017453292519943296
#define RAD_TO_DEG  57.295779513082321
#define RAD_TO_M    6.366707019493707e6

/**
 *
 */
template<typename T>
static inline T deg2rad(T deg) {
  return deg * static_cast<T>(DEG_TO_RAD);
}

/**
 *
 */
template<typename T>
static inline T rad2deg(T rad) {
  return rad * static_cast<T>(RAD_TO_DEG);
}

/**
 * Get meters from radians on earth sphere
 */
template<typename T>
static inline T rad2m(T rad) {
  return rad * static_cast<T>(RAD_TO_M);
}

/**
 *
 */
template <typename T>
T wrap_180(T error) {
  if (error > static_cast<T>(180))
    error -= static_cast<T>(360);
  else if (error < static_cast<T>(-180))
    error += static_cast<T>(360);
  return error;
}

/**
 *
 */
template <typename T>
T wrap_360(T angle) {
  if (angle > static_cast<T>(360))
    angle -= static_cast<T>(360);
  else if (angle < static_cast<T>(0))
    angle += static_cast<T>(360);
  return angle;
}

/**
 *
 */
template <typename T>
T wrap_pi(T error) {
  if (error > static_cast<T>(M_PI))
    error -= 2 * static_cast<T>(M_PI);
  else if (error < -static_cast<T>(M_PI))
    error += 2 * static_cast<T>(M_PI);
  return error;
}

/**
 *
 */
template <typename T>
T wrap_2pi(T angle) {
  if (angle > 2 * static_cast<T>(M_PI))
    angle -= 2 * static_cast<T>(M_PI);
  else if (angle < 0)
    angle += 2 * static_cast<T>(M_PI);
  return angle;
}

template <typename T>
T tangentLineCourse(const T (&lineVector)[2][1], T clockwise) {
  T course = atan2(lineVector[1][0], lineVector[0][0]);
  course += clockwise*static_cast<T>(M_PI_2);
  course = wrap_2pi(course);
  return course;
}

#endif /* GEOMETRY_HPP_ */
