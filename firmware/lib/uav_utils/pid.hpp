#ifndef PID_HPP_
#define PID_HPP_

#include "float.h" /* for FLT_EPSILON macro */
#include "iir.hpp"

#define PID_CLAMP_NEG       -1
#define PID_CLAMP_NONE      0
#define PID_CLAMP_POS       1

/**
 *
 */
template <typename T>
struct PIDInit {
  PIDInit(void) {
    memset(this, 0, sizeof(*this));
  }
  T const *P;
  T const *I;
  T const *D;
  T const *Min;
  T const *Max;
  T (*postproc)(T);
};

/**
 * This Base PID control class.
 */
template <typename T>
class PidControlBase {
public:
  PidControlBase(void) :
  postproc(nullptr),
  iState(0),
  errorPrev(0),
  pGain(nullptr),
  iGain(nullptr),
  dGain(nullptr),
  Min(nullptr),
  Max(nullptr)
  {
    iState = 0;
  }

  /**
   *
   */
  void reset(void) {
    iState = 0;
  }

  /**
   *
   */
  void start(const PIDInit<T> &init) {
    start_common(init);
  }

  /**
   * @brief   Return integrator state.
   */
  T __dbg_getiState(void) const {
    return iState;
  }

protected:
  /**
   * @brief   Pointer to postprocessing function for error. Generally is wrap_pi() for delta Yaw.
   *          Set to nullptr if unneeded.
   */
  T (*postproc)(T);
  T iState;           /* Integrator state */
  T errorPrev;        /* Previous error value for trapezoidal integration */
  T const *pGain;     /* proportional gain */
  T const *iGain;     /* integral gain */
  T const *dGain;     /* derivative gain */
  T const *Min;
  T const *Max;
  int clamping = PID_CLAMP_NONE;

  /**
   *
   */
  void start_common(const PIDInit<T> &init) {

    osalDbgCheck((nullptr != init.P) &&
                 (nullptr != init.I) &&
                 (nullptr != init.D) &&
                 (nullptr != init.Min) &&
                 (nullptr != init.Max));

    this->pGain     = init.P;
    this->iGain     = init.I;
    this->dGain     = init.D;
    this->Min       = init.Min;
    this->Max       = init.Max;
    this->postproc  = init.postproc;
  }

  /**
   * @brief   Combines all terms and gains.
   */
  T do_pid(T error, T dTerm) {

    T ret = *this->pGain * error +
            *this->iGain * this->iState +
            *this->dGain * dTerm;

    /* clamp returning value and store clamping position for next iteration */
    if (ret > *this->Max) {
      clamping = PID_CLAMP_POS;
      return *this->Max;
    }
    else if (ret < *this->Min) {
      clamping = PID_CLAMP_NEG;
      return *this->Min;
    }
    else {
      clamping = PID_CLAMP_NONE;
      return ret;
    }
  }
};

/**
 * This PID calculates derivative term using current position value
 */
template <typename T>
class PidControlSelfDerivative : public PidControlBase <T> {
public:
  /**
   *
   */
  PidControlSelfDerivative(void):
  need_filter(false){;}

  /**
   *
   */
  void start(const PIDInit<T> &init, const T *iir_a, const T *iir_b) {

    this->start_common(init);

    osalDbgCheck(((nullptr == iir_a) && (nullptr == iir_b)) ||
                 ((nullptr != iir_a) && (nullptr != iir_b)));
    if (nullptr != iir_a) {
      filter.set_taps(iir_a, iir_b);
      need_filter = true;
    }
    else
      need_filter = false;
  }

  /**
   * @brief   Update PID.
   * @param[in] current   Current position for derivative term self calculation
   * @param[in] target    Target value

   */
  T operator()(T current, T target, T dT) {

    T error = target - current;
    if (nullptr != this->postproc)
      error = this->postproc(error);

    /* calculate the integral state with appropriate limiting */
    T iPiece = (error + this->errorPrev) * dT / 2;
    if ((iPiece * this->clamping) <= 0)
      this->iState += iPiece;

    /* calculate the derivative term */
    T dTerm = (error - this->errorPrev) / dT;
    this->errorPrev = error;
    if (need_filter)
      dTerm = filter(dTerm);

    return this->do_pid(error, dTerm);
  }

private:
  filters::IIR<T, T, 1> filter;
  bool need_filter;
};

#endif /* PID_HPP_ */



