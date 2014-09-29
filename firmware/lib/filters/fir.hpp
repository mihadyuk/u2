#ifndef FIR_HPP_
#define FIR_HPP_

#include "string.h"

/**
 * @brief   FIR filter.
 * @note    set "T" and "dataT" to the same type for fastest possible code
 */
template<typename T, typename dataT, int L>
class FIR {
public:
  /**
   * @brief   Default constructor
   */
  FIR(void):
  kernel(NULL),
  tip(L)
  {
    return;
  }

  /**
   * @brief     Constructor.
   * @details   Call constructor implementation with zero as initial value
   *            for sample array.
   *
   * @param[in] tapsp           pointer to transformation core
   * @param[in] len             length of filter
   */
  FIR(const T *tapsp, size_t len):
  kernel(tapsp),
  tip(L)
  {
    ctor_impl(tapsp, len, 0);
  }

  /**
   * @brief     Constructor.
   * @details   Call constructor implementation with for sample array.
   *
   * @param[in] tapsp           pointer to transformation core
   * @param[in] len             length of filter
   * @param[in] initial_value   initial value for gapless filter start
   */
  FIR(const T *tapsp, size_t len, dataT initial_value):
  kernel(tapsp),
  tip(L)
  {
    ctor_impl(tapsp, len, initial_value);
  }

  /**
   * @brief     Change filter kernel on the fly
   *
   * @param[in] tapsp           pointer to transformation core
   * @param[in] len             length of filter
   */
  void setTaps(const T *tapsp, size_t len) {
    osalDbgCheck(L == len);
    kernel = tapsp;
  }

  /**
   * @brief     Slow filter implementation for reference and self test purpose.
   *
   * @param[in] sample    input sample
   *
   * @retval    filtered sample.
   */
  T update_reference(dataT sample){

    /* shift */
    for (size_t i=L-1; i>0; i--)
      X[i] = X[i-1];
    X[0] = sample;

    /* filter */
    T s = 0;
    for (size_t k=0; k<L; k++)
      s += X[k] * kernel[k];

    return s;
  }

  /**
   *
   */
  T update(dataT sample){
    T s;

    tip--;
    X[tip] = sample;

    s = convolution_engine(kernel, &X[tip], L - tip)
      + convolution_engine(&kernel[L - tip], X, tip);

    if(0 == tip)
      tip = L;

    return s;
  }

  /**
   * @brief   this variant works twice faster for MCU without hardware
   *          multiplication
   */
  T update_half(dataT sample){

    T s = 0;

    const size_t Nblock = (L - (L % 8)) / 2;

    /* shift */
    memmove(X, &X[1], sizeof(X) - sizeof(dataT));
    X[L-1] = sample;

    /* main filter */
    for (size_t k=0; k<Nblock; k+=4) {
      T x0, x1, x2, x3;
      T t0, t1, t2, t3;

      t0 = kernel[k];
      t1 = kernel[k+1];
      t2 = kernel[k+2];
      t3 = kernel[k+3];

      x0 = X[k+0] + X[L-1-k];
      x1 = X[k+1] + X[L-2-k];
      x2 = X[k+2] + X[L-3-k];
      x3 = X[k+3] + X[L-4-k];

      s += x0 * t0;
      s += x1 * t1;
      s += x2 * t2;
      s += x3 * t3;
    }

    /* tail */
    for (size_t k=Nblock; k<L/2; k++){
      s += (X[k] + X[L-1-k]) * kernel[k];
    }

    /* tail of tail */
    if (L % 2 == 1)
      s += X[L/2 + 1] * kernel[L/2 + 1];

    return s;
  }

private:
  /**
   * @brief     Constructor implementation.
   * @details   Performs checks and array initialization.
   */
  void ctor_impl(const T *tapsp, size_t len, dataT initial_value){

    osalDbgCheck((NULL != tapsp) && (L == len));

    T tapsum = 0;
    for (size_t i=0; i<L; i++)
      tapsum += tapsp[i];

    osalDbgAssert((tapsum > 0.999) && (tapsum < 1.001), "Kernel must be normalized");

    for (size_t i=0; i<L; i++)
      X[i] = initial_value;
  }

  /**
   * @brief     Optimized for Cortex-M4F version.
   * @note      Shift must be performed in higher level
   */
  T convolution_engine(const T *taps, dataT *x, size_t len) {

    const size_t Nblock = len & ~3; /* equivalent to len - (len % 4); */
    T s = 0;

    /* main filter */
    for (size_t k=0; k<Nblock; k+=4){
      T x0, x1, x2, x3;
      T t0, t1, t2, t3;

      x0 = x[k];
      x1 = x[k+1];
      x2 = x[k+2];
      x3 = x[k+3];

      t0 = taps[k];
      t1 = taps[k+1];
      t2 = taps[k+2];
      t3 = taps[k+3];

      s += x0 * t0;
      s += x1 * t1;
      s += x2 * t2;
      s += x3 * t3;
    }

    /* tail */
    for (size_t k=Nblock; k<len; k++){
      s += x[k] * taps[k];
    }

    return s;
  }

  /**
   * Pointer to filter core
   */
  const T *kernel;

  /**
   * Filter state
   */
  dataT X[L];

  /**
   * Buffer head for shift avoidance
   */
  size_t tip;
};

#endif /* FIR_HPP_ */
