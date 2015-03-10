#ifndef PID_HPP_
#define PID_HPP_

#include "putinrange.hpp"
#include "float.h" /* for FLT_EPSILON macro */
#include "iir.hpp"

/**
 * This Base PID control class.
 */
template <typename T>
class PidControlBase {
public:
  PidControlBase(T (*postproc)(T)) :
  postproc(postproc),
  pGain(nullptr), iGain(nullptr), dGain(nullptr)
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
  void dryRun(T i) {
    if (fabs(*iGain) < FLT_EPSILON * 10) // zero division protect
      iState = 0;
    else
      iState = i / *iGain;
  }

  /**
   *
   */
  void start(T const *pGain, T const *iGain, T const *dGain) {
    osalDbgCheck((nullptr != pGain) && (nullptr != iGain) && (nullptr != dGain));

    this->pGain = pGain;
    this->iGain = iGain;
    this->dGain = dGain;
  }

  /**
   * @brief   Return integrator state.
   */
  T __dbg_getiState(void) const {
    return iState;
  }

protected:
  /* Pointer to postprocessing function. Set to nullptr if unneeded. */
  T (*postproc)(T);
  T iState;           /* Integrator state */
  T errorPrev;        /* Previous error value for trapezoidal integration */
  T const *pGain;     /* proportional gain */
  T const *iGain;     /* integral gain */
  T const *dGain;     /* derivative gain */
  const T iMax = 1;
  const T iMin = -1;

  /**
   * @brief   Combines all terms and gains. Apply post processing
   *          function when needed.
   */
  T do_pid(T error, T dTerm) {

    T ret = *this->pGain * error +
            *this->iGain * this->iState +
            *this->dGain * dTerm;

    if (nullptr == postproc)
      return ret;
    else
      return postproc(ret);
  }
};

/**
 * This PID discourages derivative input calculation from subsequent
 * input error values.
 */
template <typename T>
class PIDControl : public PidControlBase <T> {
public:
  /**
   * @brief   Update PID.
   * @param[in] error   Current value error
   * @param[in] dTerm   Differental part _measured_ some way, NOT calculated
   * @param[in] dT      Time delta between measurement
   */
  T update(T error, T dTerm, T dT) {
    T ret;

    /* calculate the integral state with appropriate limiting */
    this->iState += (error + this->errorPrev) * dT / 2;
    this->iState  = putinrange(this->iState, this->iMin, this->iMax);
    this->errorPrev = error;

    return this->do_pid(error, dTerm);
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
  PidControlSelfDerivative(T (*postproc)(T)):
  PidControlBase<T>(postproc),
  need_filter(false){;}

  /**
   *
   */
  void start(T const *pGain, T const *iGain, T const *dGain,
             const T *iir_a, const T *iir_b) {
    osalDbgCheck((nullptr != pGain) && (nullptr != iGain) && (nullptr != dGain));

    this->pGain = pGain;
    this->iGain = iGain;
    this->dGain = dGain;

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
   * @param[in] position  Current position for derivative term self calculation
   * @param[in] target    Target value

   */
  T update(T position, T target, T dT) {

    T error = position - target;

    /* calculate the integral state with appropriate limiting */
    this->iState += (error + this->errorPrev) * dT / 2;
    this->iState  = putinrange(this->iState, this->iMin, this->iMax);
    this->errorPrev = error;

    /* calculate the derivative term */
    T dTerm = (position - this->positionPrev) * dT;
    this->positionPrev = position;
    if (need_filter)
      dTerm = filter.update(dTerm);

    return this->do_pid(error, dTerm);
  }

private:
  T positionPrev; /* Previous position value for derivative term calculation */
  filters::IIR<T, T, 1> filter;
  bool need_filter;
};

#endif /* PID_HPP_ */



