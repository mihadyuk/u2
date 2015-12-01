#pragma GCC optimize "-O2"
#pragma GCC optimize "-funroll-loops"

#include "xorshift.hpp"

using namespace prng;

/**
 *
 */
uint32_t xorshift32::next(void) {
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  return s;
}

/**
 *
 */
void xorshift32::fillBuf(uint32_t *p, size_t buf_len) {
  uint32_t x = this->s;

  for (size_t i=0; i<buf_len; i++) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    p[i] = x;
  }

  this->s = x;
}

/**
 *
 */
void xorshift32::dryRun(size_t N) {
  uint32_t x = this->s;

  for (size_t i=0; i<N; i++) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
  }

  this->s = x;
}

/**
 *
 */
uint32_t xorshift128::next(void) {
  uint32_t t = sx ^ (sx << 11);
  sx = sy;
  sy = sz;
  sz = sw;
  sw = sw ^ (sw >> 19) ^ t ^ (t >> 8);
  return sw;
}

/**
 *
 */
void xorshift128::fillBuf(uint32_t *p, size_t buf_len) {
  uint32_t x = sx, y = sy, z = sz, w = sw;

  for (size_t i=0; i<buf_len; i++) {
    uint32_t t = x ^ (x << 11);
    x = y;
    y = z;
    z = w;
    w = w ^ (w >> 19) ^ t ^ (t >> 8);
    p[i] = w;
  }

  sx = x;
  sy = y;
  sz = z;
  sw = w;
}

/**
 *
 */
void xorshift128::dryRun(size_t N) {
  uint32_t x = sx, y = sy, z = sz, w = sw;

  for (size_t i=0; i<N; i++) {
    uint32_t t = x ^ (x << 11);
    x = y;
    y = z;
    z = w;
    w = w ^ (w >> 19) ^ t ^ (t >> 8);
  }

  sx = x;
  sy = y;
  sz = z;
  sw = w;
}
