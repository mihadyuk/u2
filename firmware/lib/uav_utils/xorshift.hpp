#ifndef PRNG_XORSHIFT_HPP_
#define PRNG_XORSHIFT_HPP_

#include "stdint.h"
#include "stddef.h"

namespace prng {

/**
 *
 */
class xorshift32 {
public:
  xorshift32(uint32_t seed) : s(seed) {;}
  uint32_t next(void);
  void fillBuf(uint32_t *p, size_t buf_len);
  void dryRun(size_t N);
  void reseed(uint32_t seed) {
    s = seed;
  }
private:
  uint32_t s;
};

/**
 *
 */
class xorshift128 {
public:
  xorshift128(uint32_t x, uint32_t y, uint32_t z, uint32_t w) {
    reseed(x,y,z,w);
  }
  uint32_t next(void);
  void fillBuf(uint32_t *p, size_t buf_len);
  void dryRun(size_t N);
  void reseed(uint32_t x, uint32_t y, uint32_t z, uint32_t w) {
    sx = x;
    sy = y;
    sz = z;
    sw = w;
  }
private:
  uint32_t sx;
  uint32_t sy;
  uint32_t sz;
  uint32_t sw;
};

} //namespace

#endif /* PRNG_XORSHIFT_HPP_ */

