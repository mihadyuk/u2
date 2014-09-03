#ifndef MODULOF_HPP_
#define MODULOF_HPP_

#include "math.h"

/* mod(a, 2*pi) is the remainder you get when you divide a by 2*pi;
that is, subtract the largest multiple of 2*pi less than a from a,
and that's the answer. */
static inline float modulof(float x, float y){
  return x - y * floorf(x/y);
}

static inline double modulof(double x, double y){
  return x - y * floor(x/y);
}

#endif /* MODULOF_HPP_ */
