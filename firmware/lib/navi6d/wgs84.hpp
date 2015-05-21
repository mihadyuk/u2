#ifndef WGS84_HPP
#define WGS84_HPP
//#include <math.h>
#include "e_frame.hpp"
#include "matrix_math.hpp"

#define We_WGS84   0.00007292115      
// #define Ge_WGS84   9.7803267714
#define WIE_E      7292115e-11
#define SM_AXIS    6378137.0
#define E_SQR      0.00669437999014
#define NORMAL_GRV 9.7803253359
#define GRV_CONS   0.00193185265241
#define FLATTENING 0.00335281066475
#define M_FAKTOR   0.00344978650684

template <typename T>
void get_D(T D[3][3], T r_wgs[3][1]){
  T Mh, Nh, phi;
  phi = r_wgs[0][0];
  el_curv(Mh, Nh, r_wgs);
  D[0][0] = Mh;  D[0][1] = 0.0;         D[0][2] =  0.0;
  D[1][0] = 0.0; D[1][1] = Nh*cos(phi); D[1][2] =  0.0;
  D[2][0] = 0.0; D[2][1] = 0.0;         D[2][2] = -1.0;
}

template <typename T>
void get_Di(T Di[3][3], T r_wgs[3][1]){
  T Mh, Nh, phi;
  phi = r_wgs[0][0];
  el_curv(Mh, Nh, r_wgs);
  Di[0][0] = 1.0/Mh; Di[0][1] = 0.0;               Di[0][2] =  0.0;
  Di[1][0] = 0.0;    Di[1][1] = 1.0/(Nh*cos(phi)); Di[1][2] =  0.0;
  Di[2][0] = 0.0;    Di[2][1] = 0.0;               Di[2][2] = -1.0;
}

template <typename T>
void get_WcecWcie(T Wcec[3][1], T Wcie[3][1], T r_wgs[3][1], T v[3][1]){
  T phi, cph, sph;
  phi = r_wgs[0][0];
  cph = cos(phi);
  sph = sin(phi);
  Wcie[0][0] =  We_WGS84*cph;
  Wcie[1][0] =  0.0;
  Wcie[2][0] = -We_WGS84*sph;

  T Mh, Nh, dr[3][1], Di[3][3];
  el_curv(Mh, Nh, r_wgs);
  get_Di(Di, r_wgs);
  m_mul<T, 3, 3, 1>(dr, Di, v);
  
  Wcec[0][0] =  dr[1][0]*cph;
  Wcec[1][0] = -dr[0][0];
  Wcec[2][0] = -dr[1][0]*sph;  
}

template <typename T>
void get_Ge(T Ge[3][1], T r[3][1]){
  Ge[0][0] = 0.0;
  Ge[1][0] = 0.0;
  T p, h, sL, sL2, g1;
  p = r[0][0];
  h = r[2][0];
  sL = sin(p);
  sL2 = sL*sL;

  g1 = NORMAL_GRV*(1+GRV_CONS*sL2)/(sqrt(1.0-E_SQR*sL2));
  Ge[2][0] = g1*(1.0-(2.0/SM_AXIS)*(1.0+FLATTENING+M_FAKTOR-2.0*FLATTENING*sL2)*h+3.0*h*h/SM_AXIS/SM_AXIS);
}
  
#endif //WGS84_HPP
