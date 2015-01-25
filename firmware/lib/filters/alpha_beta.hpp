#ifndef ALPHA_BETA_H_
#define ALPHA_BETA_H_

namespace filters {

/**
 * Template of alpha-beta filter class with length changed on the fly
 */
template<typename T>
class AlphaBeta{
public:
  /**
   * default constructor
   */
  AlphaBeta(void){
    S = 0;
  };

  /**
   * Constructor resetting current value to specified one
   */
  AlphaBeta(T val, int len){
    reset(val, len);
  };

  /**
   * Perform addition of new sample to filter and return current filtered
   * result
   */
  T update(T val, int len){
    T tmp = S / len;
    S = S - tmp + val;
    return tmp;
  };

  /**
   * Return current result without updating
   */
  T get(int len){
    return S / len;
  };

  /**
   * Reset filter state
   */
  void reset(T val, int len){
    S = val * len;
  }

private:
  /**
   * Accumulator
   */
  T S;
};

/**
 * Template of alpha-beta filter class with fixed length
 */
template<typename T, int len>
class AlphaBetaFixedLen{
public:
  /**
   * Constructor resetting current value to specified one
   */
  AlphaBetaFixedLen(T val){
    chDbgCheck(len != 0, "Zero length forbidden");
    reset(val);
  };

  /**
   * Perform addition of new sample to filter and return current filtered
   * result
   */
  T update(T val){
    T tmp = S / len;
    S = S - tmp + val;
    return tmp;
  };

  /**
   * Return current result without updating
   */
  T get(void){
    return S / len;
  };

  /**
   * Return length of filter
   */
  int getLen(void){
    return len;
  };

  /**
   * Reset filter state
   */
  void reset(T val){
    S = val * len;
  }

private:
  /**
   * Accumulator
   */
  T S;
};

} /* namespace */
#endif /* ALPHA_BETA_H_ */
