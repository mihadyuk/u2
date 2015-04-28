#ifndef MATH_F_HPP_
#define MATH_F_HPP_

#include <math.h>

/**
 * Fast hardware square root
 */
__attribute__((always_inline)) __STATIC_INLINE float __VSQRT(float value)
{
  float result;
  __ASM volatile ("vsqrt.f32 %0, %1" : "=w"(result) : "w"(value));
  return(result);
}

/**
 * Overload functions for float32 arguments
 */
inline float sqrt(float v){
#if CORTEX_USE_FPU
  return __VSQRT(v);
#else
  return sqrtf(v);
#endif /* CORTEX_USE_FPU */
}

static inline float cos(float v){
  return cosf(v);
}

static inline float acos(float v){
  return acosf(v);
}

static inline float sin(float v){
  return sinf(v);
}

static inline float asin(float v){
  return asinf(v);
}

static inline float tan(float v){
  return tanf(v);
}

static inline float atan(float v){
  return atanf(v);
}

static inline float atan2(float x, float y){
  return atan2f(x, y);
}

static inline float fabs(float x){
  return fabsf(x);
}

static inline float pow(float x, float y){
  return powf(x, y);
}

#endif /* MATH_F_HPP_ */
