#ifndef WIND_ESTIMATOR_HPP
#define WIND_ESTIMATOR_HPP

#include "kalman.hpp"
#include "matrix_math.hpp"

template <typename T>
class Wind_estimator{
  public:
    Wind_estimator(void);
    bool initialize(T R, T Qm, T P, T dT_in);
    void update(T v_sns[3][1], T v_odo[3][1], T qnb_mag[4][1], bool en_flg);
    T w[3][1];
    Kalman<T,3,3,3> Filter;
  private:  
    bool initialized;
};

template <typename T>
Wind_estimator<T>::Wind_estimator(void) :
initialized(false)
{
}

template <typename T>
bool Wind_estimator<T>::initialize(T R, T Qm, T P, T dT_in){
  /* initialize R matrix.*/
  m_set<T,3,1>(Filter.R,0.0);
  for(int i = 0; i<=2; i++){
    Filter.R[i][0] = R*R;
  }
  
  /* initialize Qm matrix.*/
  m_set<T,3,3>(Filter.P,0.0);
  m_set<T,3,1>(Filter.Qm,0.0); 
  
  for(int i = 0; i<=2; i++){
    Filter.Qm[i][0] = Qm*Qm;
    Filter.P[i][i] = P*P;
  }
  /* initialize F matrix.*/
  m_eye<T,3>(Filter.F);
 /* initialize G matrix.*/
  m_eye<T,3>(Filter.G);
  
  Filter.rst(true);
  Filter.set_dT(dT_in);
  
  initialized = true;
  return true;
}

template <typename T>
void Wind_estimator<T>::update(T v_sns[3][1], T v_odo[3][1], T qnb_mag[4][1], bool en_flg){
  
  T va_ned[3][1];
  T Cnb_mag[3][3];
  quat2dcm<T>(Cnb_mag,qnb_mag);
  m_mul<T,3,3,1>(va_ned,Cnb_mag,v_odo);
  //Kalman
  //set H
  if (en_flg == true){
    m_eye<T,3>(Filter.H);
  }
  else{
    m_zeros<T,3>(Filter.H);
  }

  //set F G
  m_eye<T,3>(Filter.F);
  m_eye<T,3>(Filter.G);  
  //set Y
  m_minus<T,3,1>(Filter.Y, va_ned, v_sns);

  Filter.update(); 
  m_copy<T,3,1>(w, Filter.X);
}

#endif //WIND_ESTIMATOR_HPP
