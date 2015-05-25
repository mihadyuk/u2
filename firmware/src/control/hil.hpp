#ifndef CONTROL_HIL_HPP_
#define CONTROL_HIL_HPP_

#include "bitmap.h"

namespace control {

/**
 *
 */
constexpr size_t __words(size_t b) {
  return (b / sizeof(bitmap_word_t)) + ((b % sizeof(bitmap_word_t) > 0) ? 1 : 0);
}

/**
 *
 */
template <size_t bits>
class HILBitmap {
public:
  HILBitmap(void) {
    this->bitmap.array = this->arr;
    this->bitmap.len = __words(bits);
    bitmapObjectInit(&this->bitmap, 0);
  }
  bitmap_t bitmap;
private:
  bitmap_word_t arr[__words(bits)];
};

/**
 *
 */
class HIL {
public:
  HIL(void);
  void update(ACSInput &acs_in);
  void override(float val, state_vector_enum addr);
  void disableAll(void);
private:
  ACSInput shadow;
  HILBitmap<ACS_INPUT_ENUM_END> bmp;
};

} /* namespace */

#endif /* CONTROL_HIL_HPP_ */

