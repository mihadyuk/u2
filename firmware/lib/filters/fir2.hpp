#ifndef FIR2_HPP_
#define FIR2_HPP_

/**
 * @brief   FIR filter.
 * @note    set "T" and "dataT" to the same type for fastest possible code
 *
 * @param[in] L           filter length
 * @param[in] N           count of filter states
 */
template<typename T, typename dataT, unsigned int L, unsigned int N>
class FIR2 {
public:
  /**
   * @brief   Default constructor
   */
  FIR2(void):
  kernel(nullptr),
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
  FIR2(const T *tapsp, size_t len):
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
  FIR2(const T *tapsp, size_t len, dataT initial_value):
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
   * @param[in] sample    pointer to input sample vector
   * @param[out] result   pointer to filtered data vector.
   */
  void update_reference(T *result, const dataT *sample){

    /* shift and add new sample */
    memmove(X, &X[N], sizeof(X) - sizeof(dataT) * N);
    memcpy(&X[(L-1)*N], sample, sizeof(dataT) * N);

    /* filter */
    for (size_t n=0; n<N; n++){
      T s = 0;
      for (size_t k=0; k<L; k++)
        s += X[k*N+n] * kernel[k];
      result[n] = s;
    }
  }

  /**
   * @brief     Slow filter implementation for reference and self test purpose.
   *
   * @param[in] sample    pointer to input sample vector
   * @param[out] result   pointer to filtered data vector.
   */
  void update_half(T *result, const dataT *sample){

    /* shift and add new sample */
    memmove(X, &X[N], sizeof(X) - sizeof(dataT) * N);
    memcpy(&X[(L-1)*N], sample, sizeof(dataT) * N);

    /* filter */
    for (size_t n=0; n<N; n++){
      T s = 0;
      size_t k;
      for (k=0; k<L/2; k++)
        s += (X[k*N+n] + X[(L-1-k)*N+n]) * kernel[k];
      if (L % 2 == 1)
        s += X[k*N+n] * kernel[k];

      result[n] = s;
    }
  }

  /**
   *
   */
  void update_no_move(T *result, const dataT *sample) {

    tip--;
    memcpy(&X[tip], sample, sizeof(dataT) * N);

    /* prepare working area */
    for (size_t n=0; n<N; n++)
      result[n] = 0;

    convolution_engine(result, kernel,           &X[tip*N], L - tip);
    convolution_engine(result, &kernel[L - tip], X,         tip);

    if(0 == tip)
      tip = L;
  }

  /**
   *
   */
  void update_unroll(T *result, const dataT *sample){

    /* shift and add new sample */
    memmove(X, &X[N], sizeof(X) - sizeof(dataT) * N);
    memcpy(&X[(L-1)*N], sample, sizeof(dataT) * N);

    /* use result pointer as working area */
    for (size_t n=0; n<N; n++)
      result[n] = 0;

    /* main filter */
    const size_t Nblock = L & ~3;
    for (size_t k=0; k<Nblock; k+=4) {
      T t0, t1, t2, t3;

      t0 = kernel[k];
      t1 = kernel[k+1];
      t2 = kernel[k+2];
      t3 = kernel[k+3];

      for (size_t n=0; n<N; n++) {
        T x0, x1, x2, x3;

        x0 = X[(k+0)*N+n];
        x1 = X[(k+1)*N+n];
        x2 = X[(k+2)*N+n];
        x3 = X[(k+3)*N+n];

        result[n] += x0*t0 + x1*t1 + x2*t2 + x3*t3;
      }
    }

    /* tail */
    for (size_t k=Nblock; k<L; k++){
      for (size_t n=0; n<N; n++) {
        result[n] += X[k*N+n] * kernel[k];
      }
    }
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

    for (size_t i=0; i<L*N; i++)
      X[i] = initial_value;
  }

  /**
   *
   */
  void convolution_engine(T *ret, const T *core, const dataT *data,
                                      const size_t len){

    T tmp[N] = {0};

//    for (size_t n=0; n<N; n++)
//      tmp[n] = 0;

    /* main filter */
    const size_t Nblock = len & ~3;
    for (size_t k=0; k<Nblock; k+=4) {
      T t0, t1, t2, t3;

      t0 = core[k];
      t1 = core[k+1];
      t2 = core[k+2];
      t3 = core[k+3];

      for (size_t n=0; n<N; n++) {
        T x0, x1, x2, x3;

        x0 = data[(k+0)*N+n];
        x1 = data[(k+1)*N+n];
        x2 = data[(k+2)*N+n];
        x3 = data[(k+3)*N+n];

        tmp[n] += x0*t0 + x1*t1 + x2*t2 + x3*t3;
      }
    }

    /* tail */
    for (size_t k=Nblock; k<len; k++){
      for (size_t n=0; n<N; n++) {
        tmp[n] += data[k*N+n] * core[k];
      }
    }

    for (size_t n=0; n<N; n++)
      ret[n] = tmp[n];
  }

  /**
   * Pointer to filter kernel
   */
  const T *kernel;

  /**
   * Filter state(s)
   */
  dataT X[L*N];

  /**
   * Buffer head for shift avoidance
   */
  size_t tip;
};

#endif /* FIR2_HPP_ */
