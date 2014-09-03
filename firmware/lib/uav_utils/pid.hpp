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
  PidControlBase(void){
    iState = 0;
  }

  /**
   *
   */
  void reset(void){
    iState = 0;
  }

  /**
   *
   */
  void dryRun(T i){
    if (fabs(*iGain) < FLT_EPSILON * 10) // zero divizion protect
      iState = 0;
    else
      iState = i / *iGain;
  }

  /**
   *
   */
  void start(T const *iMin,  T const *iMax,
             T const *pGain, T const *iGain, T const *dGain){
    this->iMin  = iMin;
    this->iMax  = iMax;
    this->pGain = pGain;
    this->iGain = iGain;
    this->dGain = dGain;
  }

  /**
   * @brief   Update PID.
   *
   */
  virtual T update(T error, T diff_or_position) = 0;

  /**
   * @brief   Return integrator state.
   */
  T dbg_getiState(void) const {
    return iState;
  }

protected:
  T iState;           /* Integrator state */
  T const *iMax;
  T const *iMin;      /* Maximum and minimum allowable integrator state */
  T const *iGain;     /* integral gain */
  T const *pGain;     /* proportional gain */
  T const *dGain;     /* derivative gain */
};

/**
 * This PID calculates derivative term using current position value
 */
template <typename T>
class PidControlSelfDerivative : public PidControlBase <T> {
public:
  /**
   * @brief   Update PID.
   * @param[in] error     Current value error
   * @param[in] position  Current position for derivative term self calculation
   */
  T update(T error, T position){
    T pTerm, dTerm, iTerm;

    /* calculate the proportional term */
    pTerm = *this->pGain * error;

    /* calculate the integral state with appropriate limiting */
    this->iState += error;
    this->iState  = putinrange(this->iState, *this->iMin, *this->iMax);

    /* calculate the integral term */
    iTerm = *this->iGain * this->iState;

    /* calculate the derivative term */
    dTerm = *this->dGain * (position - this->dState);
    this->dState = position;

    return pTerm + iTerm + dTerm;
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
   * @param[in] diff    Differental part _measured_ some way, NOT calculated
   */
  T update(T error, T diff){
    /* calculate the integral state with appropriate limiting */
    this->iState += error;
    this->iState  = putinrange(this->iState, *this->iMin, *this->iMax);

    return (*this->pGain * error) + (*this->iGain * this->iState) + (*this->dGain * diff);
  }
};

#endif /* PID_HPP_ */


