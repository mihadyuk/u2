#ifndef TUMBLER_HPP_
#define TUMBLER_HPP_

#include "hysteresis.hpp"

/**
 * @brief     Two positional tumbler
 */
template <typename T, T min0, T max0, T min1, T max1>
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
    static_assert((max0 < min1), "");
    osalDbgCheck(val < 2);
    prev = val;
  }

  unsigned int prev;
  Hysteresis<T, min0, max0> h0;
  Hysteresis<T, min1, max1> h1;
};

/**
 * @brief     Three positional tumbler
 */
template <typename T, T min0, T max0, T min1, T max1, T min2, T max2>
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
    static_assert((max0 < min1) && (max1 < min2), "There must be some room between hysteresises");
    osalDbgCheck(val < 3);                      //Incorrect initial value
    prev = val;
  }

  unsigned int prev;
  Hysteresis<T, min0, max0> h0;
  Hysteresis<T, min1, max1> h1;
  Hysteresis<T, min2, max2> h2;
};

#endif /* TUMBLER_HPP_ */
