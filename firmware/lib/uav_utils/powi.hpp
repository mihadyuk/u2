#ifndef POWI_HPP_
#define POWI_HPP_

/**
 * Integer power function.
 */
template <typename T>
T powi(T base, uint32_t exp){
  T result = 1;
  while (exp > 0){
    result *= base;
    exp--;
  }
  return result;
}

#endif /* POWI_HPP_ */
