#ifndef PID_HPP_
#define PID_HPP_

#include "putinrange.hpp"
#include "float.h" /* for FLT_EPSILON macro */

/**
 * This Base PID control class.
 */
template <typename T>
class PidControlBase {
public:
  PidControlBase(void) : pGain(nullptr), iGain(nullptr), dGain(nullptr) {
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
    if (fabs(*iGain) < FLT_EPSILON * 10) // zero divizion protect
      iState = 0;
    else
      iState = i / *iGain;
  }

  /**
   *
   */
  void start(T const *pGain, T const *iGain, T const *dGain, T iMin, T iMax) {

    osalDbgCheck(iMax > iMin);

    this->pGain = pGain;
    this->iGain = iGain;
    this->dGain = dGain;
    this->iMin = iMin;
    this->iMax = iMax;
  }

  /**
   * @brief   Return integrator state.
   */
  T __dbg_getiState(void) const {
    return iState;
  }

protected:
  T iState;           /* Integrator state */
  T errorPrev;        /* Previous error value for trapezoidal integration */
  T const *pGain;     /* proportional gain */
  T const *iGain;     /* integral gain */
  T const *dGain;     /* derivative gain */
  T iMax;
  T iMin;
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
    /* calculate the integral state with appropriate limiting */
    this->iState += (error + this->errorPrev) * dT / 2;
    this->iState  = putinrange(this->iState, this->iMin, this->iMax);
    this->errorPrev = error;

    return (*this->pGain * error) + (*this->iGain * this->iState) + (*this->dGain * dTerm);
  }
};

/**
 * This PID calculates derivative term using current position value
 */
template <typename T>
class PidControlSelfDerivative : public PidControlBase <T> {
public:
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

    return (*this->pGain * error) + (*this->iGain * this->iState) + (*this->dGain * dTerm);
  }

private:
  T positionPrev; /* Previous position value for derivative term calculation */
};

#endif /* PID_HPP_ */



