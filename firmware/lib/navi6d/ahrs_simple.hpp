#ifndef AHRS_SIMPLE_HPP
#define AHRS_SIMPLE_HPP

#include "att.hpp"
#include "matrix_math.hpp"

template <typename T>
class Ahrs_simple{
  public:
    Ahrs_simple(void); 
    void simple_update(T av[3][1], T mv[3][1]);
    void mag_heading_update(T qnv_in[4][1], T mb[3][1], T qbv[4][1]);
    T qnv[4][1];
    T eu[3][1];
    
    T qnv_mag[4][1];
    T eu_mag[3][1];
    
    T mag_dec;
};

template <typename T>
Ahrs_simple<T>::Ahrs_simple(void)
{
  m_set<T,4,1>(qnv,0.0);
  qnv[0][0] = 1.0;
  m_set<T,4,1>(qnv_mag,0.0);
  qnv_mag[0][0] = 1.0;
  mag_dec = 0.0;
}

template <typename T>
void Ahrs_simple<T>::mag_heading_update(T qnv_in[4][1], T mb[3][1], T qbv[4][1]){
  T av[3][1];
  T mv[3][1];
  T Cnv[3][3];
  T Cvb[3][3];
  T qvb[4][1];
  qcon<T>(qvb,qbv);
  quat2dcm<T>(Cnv,qnv_in);
  quat2dcm<T>(Cvb,qvb);
  m_mul<T,3,3,1>(mv,Cvb,mb);
  av[0][0] = -Cnv[2][0];
  av[1][0] = -Cnv[2][1];
  av[2][0] = -Cnv[2][2];
  this->simple_update(av, mv);
}

template <typename T>
void Ahrs_simple<T>::simple_update(T av[3][1], T mv[3][1]){
  T D[3][1], mag[3][1];
  T Cvn[3][3];
      
//   %   Dv = -acc;
// %   Dv = Dv/norm(Dv);
// %   Ev = cross(Dv,mag(:,i)/norm(mag(:,i)));
// %   Ev = Ev/norm(Ev);
// %   Nv = -cross(Dv, Ev);
// %   Nv = Nv/norm(Nv);
// %   C = [Nv Ev Dv];

  m_copy<T,3,1>(D, av);
  m_copy<T,3,1>(mag, mv);
  m_norm<T,3>(D);
  m_norm<T,3>(mag);
  m_mul_s<T,3,1>(D, D, -1.0);

  set_sub<T,3,3,3,1>(Cvn, D, 0, 2, 1.0);  
  
  T crD[3][3];
  T E[3][1];
  cpm<T>(crD, D);
  m_mul<T,3,3,1>(E, crD, mag);
  m_norm<T,3>(E);
  set_sub<T,3,3,3,1>(Cvn, E, 0, 1, 1.0);
  
  //N=D
  m_mul<T,3,3,1>(D, crD, E);
  m_norm<T,3>(D);
  set_sub<T,3,3,3,1>(Cvn, D, 0, 0, -1.0);
  m_tran<T,3,3>(crD,Cvn);
  
  dcm2quat<T>(qnv_mag, crD);  
  //declination correction
  T qdec[4][1];
  m_set<T,4,1>(qdec,0.0);  
  qdec[0][0] = cos(0.5*mag_dec);
  qdec[3][0] = sin(0.5*mag_dec);
  qmult<T>(qnv, qdec, qnv_mag);
  
  //to euler
  quat2euler<T>(eu_mag, qnv_mag);
  quat2euler<T>(eu, qnv);
  //
}

#endif //AHRS_SIMPLE_HPP
