#ifndef TUMBLER_HPP_
#define TUMBLER_HPP_

#include "hysteresis.hpp"

/**
 * @brief     Two positional tumbler
 */
template <typename T, T v0, T v1, T tolerance>
class Tumbler2 {
public:
  Tumbler2(void){
    __ctor(0);
  }

  Tumbler2(T val){
    __ctor(val);
  }

  unsigned int get(void){
    return prev;
  }

  unsigned int update(T val){
    if (0 == h0.check(val))
      prev = 0;
    else if (0 == h1.check(val))
      prev = 1;
    return prev;
  }

private:
  void __ctor(T val){
    static_assert(v0 < v1, "");
    static_assert((v0+tolerance) < (v1 - tolerance),
                  "There must be some room between hysteresises");
    osalDbgCheck(val < 2);
    prev = val;
  }

  unsigned int prev;
  Hysteresis<T, v0-tolerance, v0+tolerance> h0;
  Hysteresis<T, v1-tolerance, v1+tolerance> h1;
};

/**
 * @brief     Three positional tumbler
 */
template <typename T, T v0, T v1, T v2, T tolerance>
class Tumbler3 {
public:
  Tumbler3(void){
    __ctor(0);
  }

  Tumbler3(T val){
    __ctor(val);
  }

  unsigned int get(void){
    return prev;
  }

  unsigned int update(T val){
    if (0 == h0.check(val))
      prev = 0;
    else if (0 == h1.check(val))
      prev = 1;
    else if (0 == h2.check(val))
      prev = 2;
    return prev;
  }

private:
  void __ctor(T val){
    static_assert((v0 < v1) && (v1 < v2), "");
    static_assert((v0+tolerance) < (v1 - tolerance),
                  "There must be some room between hysteresises");
    static_assert((v1+tolerance) < (v2 - tolerance),
                  "There must be some room between hysteresises");
    osalDbgCheck(val < 3);                      //Incorrect initial value
    prev = val;
  }

  unsigned int prev;
  Hysteresis<T, v0-tolerance, v0+tolerance> h0;
  Hysteresis<T, v1-tolerance, v1+tolerance> h1;
  Hysteresis<T, v2-tolerance, v2+tolerance> h2;
};

#endif /* TUMBLER_HPP_ */
