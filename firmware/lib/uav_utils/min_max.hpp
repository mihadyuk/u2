#ifndef MIN_MAX_HPP_
#define MIN_MAX_HPP_

/**
 *
 */
template<typename T>
T min(T v1, T v2) {

  if (v1 < v2)
    return v1;
  else
    return v2;
}

template<typename T>
T min(T v1, T v2, T v3) {
  return min(min(v1, v2), v3);
}

template<typename T>
T min(T v1, T v2, T v3, T v4) {
  return min(min(v1, v2, v3), v4);
}

/**
 *
 */
template<typename T>
T max(T v1, T v2) {

  if (v1 > v2)
    return v1;
  else
    return v2;
}

template<typename T>
T max(T v1, T v2, T v3) {
  return max(max(v1, v2), v3);
}

template<typename T>
T max(T v1, T v2, T v3, T v4) {
  return max(max(v1, v2, v3), v4);
}

/**
 *
 */
template<typename T>
constexpr T max_const(T v1, T v2) {
  return ((v1 > v2) ? v1 : v2);
}

template<typename T>
constexpr T max_const(T v1, T v2, T v3) {
  return max_const(max_const(v1, v2), v3);
}

template<typename T>
constexpr T max_const(T v1, T v2, T v3, T v4) {
  return max_const(max_const(v1, v2, v3), v4);
}

/**
 *
 */
template<typename T>
constexpr T min_const(T v1, T v2) {
  return ((v1 < v2) ? v1 : v2);
}

template<typename T>
constexpr T min_const(T v1, T v2, T v3) {
  return min_const(min_const(v1, v2), v3);
}

template<typename T>
constexpr T min_const(T v1, T v2, T v3, T v4) {
  return min_const(min_const(v1, v2, v3), v4);
}

#endif /* MIN_MAX_HPP_ */
