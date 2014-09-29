#ifndef FIR_HPP_
#define FIR_HPP_

/**
 * @brief   FIR filter.
 * @note    set "T" and "dataT" the same type for fastest possible code
 */
template<typename T, typename dataT, int N>
class FIR {
public:
  /**
   * @brief   Default constructor
   */
  FIR(void):
  kernel(NULL),
  tip(N)
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
  tip(N)
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
  tip(N)
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
    osalDbgCheck(N == len);
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
    for (size_t i=N-1; i>0; i--)
      X[i] = X[i-1];
    X[0] = sample;

    /* filter */
    T s = 0;
    for (size_t k=0; k<N; k++)
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

    s = convolution_engine(kernel, &X[tip], N - tip)
      + convolution_engine(&kernel[N - tip], X, tip);

    if(0 == tip)
      tip = N;

    return s;
  }

private:
  /**
   * @brief     Constructor implementation.
   * @details   Performs checks and array initialization.
   */
  void ctor_impl(const T *tapsp, size_t len, dataT initial_value){

    osalDbgCheck((NULL != tapsp) && (N == len));

    T tapsum = 0;
    for (size_t i=0; i<N; i++)
      tapsum += tapsp[i];

    osalDbgAssert((tapsum > 0.999) && (tapsum < 1.001), "Kernel must be normalized");

    for (size_t i=0; i<N; i++)
      X[i] = initial_value;
  }

  /**
   * @brief     Optimized for Cortex-M4F version.
   * @note      Shift must be performed in higher level
   */
  T convolution_engine(const T *taps, dataT *x, size_t len) {

    //const size_t Nblock = len - (len % 4);
    const size_t Nblock = len & ~3;
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
  dataT X[N];

  /**
   * Buffer head for shift avoidance
   */
  size_t tip;
};

#endif /* FIR_HPP_ */
