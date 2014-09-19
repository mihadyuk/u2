#ifndef POLYNOMIAL_HPP_
#define POLYNOMIAL_HPP_

/**
 * @brief       Multiplying value by polynomial.
 *
 * @param[in] poly    Array of polynomial coefficients (MSB first). For example
 *                    polynomial x^3 + 3x^2 + 12
 *                    must be represented as p[4] = {1, 3, 0, 12}
 * @param[in] len     Length of polynomial array
 * @param[in] val     This value will be multiplied by polynomial
 *
 * @return            Result of multiplication.
 */
template <typename T>
T PolyMul(const T *poly, size_t len, T val){
  T result;

  len--;
  result = poly[len];

  if (0 == len)
    return result;

  while (len--){
    result += poly[len] * val;
    val *= val;
  }

  return result;
}

/**
 * @brief   Convenient class
 */
template <typename T>
class Polynomial {
public:
  Polynomial(const T *poly, size_t len){
    osalDbgAssert(len > 0, "polynomial length can not be zero");
    this->poly = poly;
    this->len = len;
  }
  /**
   * @brief Overloaded multiplication operator
   */
  T operator*(T val) const {
    return PolyMul(poly, len, val);
  }
private:
  const T *poly;
  size_t len;
};

/**
 * @brief Overloaded polynomial multiplication operator for reverse operand order
 */
template <typename T>
T operator*(T val, const Polynomial<T> &poly){
  return poly * val;
}

#endif /* POLYNOMIAL_HPP_ */
