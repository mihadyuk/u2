#ifndef MATH_F_HPP_
#define MATH_F_HPP_

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

inline float cos(float v){
  return cosf(v);
}

inline float acos(float v){
  return acosf(v);
}

inline float sin(float v){
  return sinf(v);
}

inline float asin(float v){
  return asinf(v);
}

inline float tan(float v){
  return tanf(v);
}

inline float atan(float v){
  return atanf(v);
}

inline float atan2(float x, float y){
  return atan2f(x, y);
}

inline float fabs(float x){
  return fabsf(x);
}

inline float pow(float x, float y){
  return powf(x, y);
}

#endif /* MATH_F_HPP_ */
