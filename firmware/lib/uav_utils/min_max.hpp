#ifndef MIN_MAX_HPP_
#define MIN_MAX_HPP_

/**
 *
 */
template<typename T>
T min(T v1, T v2){

  if (v1 < v2)
    return v1;
  else
    return v2;
}

/**
 *
 */
template<typename T>
T max(T v1, T v2){

  if (v1 > v2)
    return v1;
  else
    return v2;
}

#endif /* MIN_MAX_HPP_ */
