#ifndef QMATH_HPP
#define QMATH_HPP
#include "matrix_math.hpp"
#include <math.h>

template<typename T>
void cpm(T skew[3][3], T v[3][1]){
  T v_x, v_y, v_z;
  v_x = v[0][0];
  v_y = v[1][0];
  v_z = v[2][0];
  skew[0][0] =  0.0; skew[0][1] = -v_z; skew[0][2] =  v_y;
  skew[1][0] =  v_z; skew[1][1] =  0.0; skew[1][2] = -v_x;
  skew[2][0] = -v_y; skew[2][1] =  v_x; skew[2][2] =  0.0;
}

template<typename T>
void qnorm(T q[4][1]){
  T q_n;
  q_n = sqrt(q[0][0]*q[0][0]+q[1][0]*q[1][0]+q[2][0]*q[2][0]+q[3][0]*q[3][0]);
  for (int i = 0; i<4; i++)
  {
    q[i][0] /= q_n;
  }
}

template<typename T>
void qmult(T q_o[4][1], T q1[4][1], T q2[4][1]){
  T M[4][4];
  M[0][0] = q1[0][0]; M[0][1] = -q1[1][0]; M[0][2] = -q1[2][0]; M[0][3] = -q1[3][0];
  M[1][0] = q1[1][0]; M[1][1] =  q1[0][0]; M[1][2] = -q1[3][0]; M[1][3] =  q1[2][0];
  M[2][0] = q1[2][0]; M[2][1] =  q1[3][0]; M[2][2] =  q1[0][0]; M[2][3] = -q1[1][0];
  M[3][0] = q1[3][0]; M[3][1] = -q1[2][0]; M[3][2] =  q1[1][0]; M[3][3] =  q1[0][0];
  m_mul<T,4,4,1>(q_o, M, q2);
}

template<typename T>
void qcon(T q_o[4][1], T q_i[4][1]){
  q_o[0][0] = q_i[0][0];
  for (int i = 1; i<=3; i++)
  {
    q_o[i][0] = -q_i[i][0];
  }
}

template<typename T>
void vec2quat(T q_o[4][1], T v[3][1]){
  q_o[0][0] = 0.0;
  for (int i = 1; i<=3; i++)
  {
    q_o[i][0] = v[i-1][0];
  }
}

#endif //QMATH_HPP
