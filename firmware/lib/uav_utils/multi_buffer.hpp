#ifndef MULTI_BUFFER_HPP_
#define MULTI_BUFFER_HPP_

template <typename T, int Len, int Count>
class MultiBuffer {
public:

  /**
   *
   */
  MultiBuffer(void){
    constructor_impl(0);
  }

  /**
   *
   */
  MultiBuffer(int pattern){
    constructor_impl(pattern);
  }

  /**
   *
   */
  T *next(void){
    head++;
    if (head == Count)
      head = 0;
    return &(internal_buf[head * Len]);
  }

  /**
   *
   */
  T *current(void){
    return &(internal_buf[head * Len]);
  }

  /**
   *
   */
  int len(void){
    return Len;
  }

  /**
   *
   */
  int cnt(void){
    return Count;
  }

private:
  /**
   *
   */
  void constructor_impl(int pattern){
    static_assert(Count > 1, "Multibuffer with zero count value is pointless");
    memset(internal_buf, pattern, sizeof(internal_buf));
  }

  T internal_buf[Count * Len];
  typeof(Count) head;
};

#endif /* MULTI_BUFFER_HPP_ */





