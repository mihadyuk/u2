#ifndef ATT_HPP
#define ATT_HPP
#include <math.h>
#include "qmath.hpp"
#include "matrix_math.hpp"
#define PI_att 3.141592653589793
#define c2pi_att 6.283185307179586

template<typename T>
void quat2dcm(T Cnb[3][3], T qnb[4][1]){
  T q1, q2, q3, q4;
  T q1_2, q2_2, q3_2, q4_2;
  T q23, q14, q24, q34, q12, q13;
  q1 = qnb[0][0];
  q2 = qnb[1][0];
  q3 = qnb[2][0];
  q4 = qnb[3][0];

  q1_2 = q1*q1; q2_2 = q2*q2; q3_2 = q3*q3; q4_2 = q4*q4;
  q23 = 2*q2*q3;
  q14 = 2*q1*q4;
  q24 = 2*q2*q4;
  q34 = 2*q3*q4;
  q12 = 2*q1*q2;
  q13 = 2*q1*q3;
  Cnb[0][0] = q1_2+q2_2-q3_2-q4_2;
  Cnb[1][1] = q1_2-q2_2+q3_2-q4_2;
  Cnb[2][2] = q1_2-q2_2-q3_2+q4_2;
  
  Cnb[0][1] = q23-q14;
  Cnb[0][2] = q24+q13;
  Cnb[1][0] = q23+q14;
  Cnb[1][2] = q34-q12;
  Cnb[2][0] = q24-q13;
  Cnb[2][1] = q34+q12;
}

template<typename T>
void quat2euler(T eu[3][1], T qnb[4][1]){
  T Cnb31, Cnb32, Cnb33, Cnb21, Cnb11;
  T q1, q2, q3, q4;
  T q1_2, q2_2, q3_2, q4_2;

  q1 = qnb[0][0];
  q2 = qnb[1][0];
  q3 = qnb[2][0];
  q4 = qnb[3][0];

  q1_2 = q1*q1; 
  q2_2 = q2*q2; 
  q3_2 = q3*q3; 
  q4_2 = q4*q4;
    
  Cnb11 = q1_2+q2_2-q3_2-q4_2;
  Cnb21 = 2*(q2*q3+q1*q4);
  Cnb31 = 2*(q2*q4-q1*q3);
  Cnb32 = 2*(q3*q4+q1*q2);
  Cnb33 = q1_2-q2_2-q3_2+q4_2;;

  eu[0][0] = atan2(Cnb32,Cnb33);                               //roll
  eu[1][0] = atan2(-Cnb31,sqrt(Cnb32*Cnb32 + Cnb33*Cnb33));     //theta
  eu[2][0] = atan2(Cnb21,Cnb11);                               //psi

  /* wrap 2pi */
  if (eu[2][0] < 0){
    eu[2][0] += static_cast<T>(c2pi_att);
  }
}


template<typename T>
void quat2pos(T r[3][1], T qen[4][1]){
  T pi025;
  pi025 = static_cast<T>(PI_att)/4;
  r[0][0] = -2*(atan2(qen[2][0],qen[0][0])+pi025);
  r[1][0] = 2*atan2(qen[3][0],qen[0][0]);
  r[2][0] = 0;
}

template<typename T>
void pos2quat(T qen[4][1], T r[3][1]){
  T pi025, phi05, lam05, s1, c1, s2, c2;
  pi025 = static_cast<T>(PI_att)/4;
  phi05 = r[0][0]/2;
  lam05 = r[1][0]/2;
  s1 = sin(-pi025-phi05);
  c1 = cos(-pi025-phi05);
  s2 = sin(lam05);
  c2 = cos(lam05);
  qen[0][0] =  c1*c2;
  qen[1][0] = -s1*s2;
  qen[2][0] =  s1*c2;
  qen[3][0] =  c1*s2;
}

template<typename T>
void rot2quat(T q[4][1], T phi[3][1]){
  T np;
  T n_phi05; 
  T phi_n[3][1];
  T cos_phi05, sin_phi05;
  np = sqrt(phi[0][0]*phi[0][0]+phi[1][0]*phi[1][0]+phi[2][0]*phi[2][0]);
  n_phi05 = static_cast<T>(0.5)*np;   
  m_mul_s<T,3,1>(phi_n,phi,1/np);
  if (np == 0)
  {
    q[0][0] = 1;   
    q[1][0] = 0;
    q[2][0] = 0;
    q[3][0] = 0;
  }
  else
  {
    cos_phi05 = cos(n_phi05);
    sin_phi05 = sin(n_phi05);
    q[0][0] = cos_phi05;
    q[1][0] = sin_phi05*phi_n[0][0];
    q[2][0] = sin_phi05*phi_n[1][0];
    q[3][0] = sin_phi05*phi_n[2][0];
  }
}
template<typename T>
void ahrs_simple(T qnb[4][1], T ab[3][1], T mb[3][1]){
  T acc[3][1], mag[3][1];
  T Cbn[3][3];
      
  m_copy<T,3,1>(acc, ab);
  m_copy<T,3,1>(mag, mb);
  m_norm<T,3>(acc);
  m_norm<T,3>(mag);
  m_mul_s<T,3,1>(acc, acc, -1);

//   Y = cross(Z,mag);
//   Y = Y/norm(Y);
//   Cbn(:,2) = Y;
  
//   X = -cross(Z,Y);
//   X = X/norm(X);
//   Cbn(:,1) = X;
//   Cnb = Cbn.';

  set_sub<T,3,3,3,1>(Cbn, acc, 0, 2, 1);  
  
  T tmp_33[3][3];
  T tmp[3][1];
  cpm<T>(tmp_33, acc);
  m_mul<T,3,3,1>(tmp, tmp_33, mag);
  m_norm<T,3>(tmp);
  set_sub<T,3,3,3,1>(Cbn, tmp, 0, 1, 1);
  
  m_mul<T,3,3,1>(acc, tmp_33, tmp);
  m_norm<T,3>(acc);
  set_sub<T,3,3,3,1>(Cbn, acc, 0, 0, -1);
  m_tran<T,3,3>(tmp_33,Cbn);
  
  
  dcm2quat(qnb,tmp_33);
  //eu = ATT.quat2euler(qnb);   
}

template <typename T>
static void dcm2quat(T qnb[4][1], T Cnb[3][3]){
  T Cnb11, Cnb12, Cnb13;
  T Cnb21, Cnb22, Cnb23;
  T Cnb31, Cnb32, Cnb33;
  T trC;
  T P[4];
  T mP;
  int sw;

  Cnb11 = Cnb[0][0]; Cnb12 = Cnb[0][1]; Cnb13 = Cnb[0][2];
  Cnb21 = Cnb[1][0]; Cnb22 = Cnb[1][1]; Cnb23 = Cnb[1][2];
  Cnb31 = Cnb[2][0]; Cnb32 = Cnb[2][1]; Cnb33 = Cnb[2][2];

  trC = Cnb11 + Cnb22 + Cnb33;

  P[0] = 1+trC;
  P[1] = 1+2*Cnb11-trC;
  P[2] = 1+2*Cnb22-trC;
  P[3] = 1+2*Cnb33-trC;

  //mP = max([P1 P2 P3 P4]);
  mP = P[0];
  sw = 0;
  for (int i = 1; i<4; i++){
    if (mP<P[i]){
      mP = P[i];
      sw = i;
    }
  }

  switch (sw){
    case 0:
      qnb[0][0] = sqrt(P[0])/2;
      qnb[1][0] = (Cnb32-Cnb23)/(4*qnb[0][0]);
      qnb[2][0] = (Cnb13-Cnb31)/(4*qnb[0][0]);
      qnb[3][0] = (Cnb21-Cnb12)/(4*qnb[0][0]);
    break;
    case 1:
      qnb[1][0] = sqrt(P[1])/2;
      qnb[2][0] = (Cnb21+Cnb12)/(4*qnb[1][0]);
      qnb[3][0] = (Cnb13+Cnb31)/(4*qnb[1][0]);
      qnb[0][0] = (Cnb32-Cnb23)/(4*qnb[1][0]);
    break;
    case 2:
      qnb[2][0] = sqrt(P[2])/2;
      qnb[3][0] = (Cnb32+Cnb23)/(4*qnb[2][0]);
      qnb[0][0] = (Cnb13-Cnb31)/(4*qnb[2][0]);
      qnb[1][0] = (Cnb21+Cnb12)/(4*qnb[2][0]);
    break;
    case 3:
      qnb[3][0] = sqrt(P[3])/2;
      qnb[0][0] = (Cnb21-Cnb12)/(4*qnb[3][0]);
      qnb[1][0] = (Cnb13+Cnb31)/(4*qnb[3][0]);
      qnb[2][0] = (Cnb32+Cnb23)/(4*qnb[3][0]);
    break;
    default:
    break;
  }
  m_norm<T,4>(qnb);


  if (qnb[0][0] < 0){
    for (int i=0; i<=3; i++){
      qnb[i][0] = -qnb[i][0];
    }
  }
}

template <typename T>
static void euler2quat(T q[4][1], T eu[3][1]){

  T phi05, theta05, psi05;
  T cph05, sph05, cth05, sth05, cp05, sp05;

  phi05   = eu[0][0]/2;
  theta05 = eu[1][0]/2;
  psi05   = eu[2][0]/2;
  cph05   = cos(phi05);
  sph05   = sin(phi05);
  cth05   = cos(theta05);
  sth05   = sin(theta05);
  cp05    = cos(psi05);
  sp05    = sin(psi05);

  q[0][0] = cph05*cth05*cp05 + sph05*sth05*sp05;
  q[1][0] = sph05*cth05*cp05 - cph05*sth05*sp05;
  q[2][0] = cph05*sth05*cp05 + sph05*cth05*sp05;
  q[3][0] = cph05*cth05*sp05 - sph05*sth05*cp05;

  qnorm(q);

  // normalize and negate if q0<0
  if (q[0][0] < 0){
    for (int i=0; i<=3; i++) {
      q[i][0] = -q[i][0];
    }
  }
}
#endif //ATT_HPP
