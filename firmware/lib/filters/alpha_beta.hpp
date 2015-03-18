#ifndef ALPHA_BETA_H_
#define ALPHA_BETA_H_

namespace filters {

/**
 * Template of alpha-beta filter class with fixed length
 */
template<typename T, unsigned int L>
class AlphaBeta {
public:
  /**
   * Constructor resetting current value to specified one
   */
  AlphaBeta(void) {
    static_assert(L != 0, "Zero length forbidden");
    reset(0);
  };

  /**
   * Constructor resetting current value to specified one
   */
  AlphaBeta(T val) {
    static_assert(L != 0, "Zero length forbidden");
    reset(val);
  };

  /**
   * Perform addition of new sample to filter and return current filtered
   * result
   */
  T operator() (T val) {
    T tmp = S / static_cast<T>(L);
    S = S - tmp + val;
    return tmp;
  }

  /**
   * Return current result without updating
   */
  T get(void) {
    return S / static_cast<T>(L);
  };

  /**
   * Return length of filter
   */
  typeof(L) getLen(void) {
    return L;
  };

  /**
   * Reset filter state
   */
  void reset(T val) {
    S = val * static_cast<T>(L);
  }

private:
  /**
   * Accumulator
   */
  T S;
};

/**
 * Template of alpha-beta filter class with length changed on the fly
 */
template<typename T>
class AlphaBetaVariableLen {
public:
  /**
   * default constructor
   */
  AlphaBetaVariableLen(void) {
    S = 0;
  };

  /**
   * Constructor resetting current value to specified one
   */
  AlphaBetaVariableLen(T val, unsigned L) {
    reset(val, L);
  };

  /**
   * Perform addition of new sample to filter and return current filtered
   * result
   */
  T operator() (T val, unsigned L) {
    T tmp = S / static_cast<T>(L);
    S = S - tmp + val;
    return tmp;
  };

  /**
   * Return current result without updating
   */
  T get(unsigned L) {
    return S / static_cast<T>(L);
  };

  /**
   * Reset filter state
   */
  void reset(T val, unsigned L) {
    S = val * static_cast<T>(L);
  }

private:
  /**
   * Accumulator
   */
  T S;
};

} /* namespace */
#endif /* ALPHA_BETA_H_ */
