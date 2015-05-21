#ifndef KALMAN_HPP
#define KALMAN_HPP
#include "att.hpp"
#include "matrix_math.hpp"
#include "qmath.hpp"
#include "wgs84.hpp"
#include <math.h>

#define K_TYPE
template <typename T, int nX, int nY, int nQm>
class Kalman{
public:
  Kalman(void);
  void rst(bool rst);
  void set_F(T F_in[nX][nX]);
  void set_P(T P_in[nX][nX]);
  void set_H(T H_in[nY][nX]);
  void set_Q(void);
  void set_G(T G_in[nX][nQm]);
  void set_dT(T dT_in);
  void set_Y(T Y_in[nY][1]);  
  void get_X(T X_out[nX][1]);
  void get_F(T F_out[nX][nX]);
  void get_P(T P_out[nX][nX]);
  void get_Q(T Q_out[nX][nX]);
    
  void smooth_X_seq(int i);
  void smooth_X(void);
  void smooth_P_seq(int i);
  void smooth_P(void);
  void predict_P(void);
  void predict_X(void);
  void update(void);
  bool check_P(void);
  
  T X[nX][1];
  T F[nX][nX];
  T P[nX][nX];
  T H[nY][nX];
  T R[nY][1];
  T Qm[nQm][1];
  T G[nX][nQm];
  T Y[nY][1];
  T K[nX][nY];
  T S[nY][nY];
  T iS[nY][nY];
  T Q[nX][nX];
  T dT;
  
  
private: 
  void set_Yhat(void);
  void set_Z(void);
  void set_K_seq(int i);
  void set_K(void);
  T M[nX][nX];
  T Yhat[nY][1];
  T Z[nY][1];
  T Kseq[nX][1];
};

template <typename T, int nX, int nY, int nQm>
Kalman<T, nX, nY, nQm>::Kalman(void){
}

template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::rst(bool rst){
  if (rst == true){
    m_set<T,nX,1>(X,0.0);  
  }
}

//----------
//X
//----------

template <typename T, int nX, int nY, int nQm>  
void Kalman<T, nX, nY, nQm>::get_X(T X_out[nX][1]){
  for (int i = 0; i<nX; i++){
    X_out[i][0] = X[i][0];
  }
}

//----------
//P
//----------
template <typename T, int nX, int nY, int nQm>  
void Kalman<T, nX, nY, nQm>::set_P(T P_in[nX][nX]){
  for (int i = 0; i<nX; i++){
    for (int j = 0; j<nX; j++){
      P[i][j] = P_in[i][j];
    }
  }
}


template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::get_P(T P_out[nX][nX]){
  for (int i = 0; i<nX; i++){
    for (int j = 0; j<nX; j++){
      P_out[i][j] = P[i][j];
    }
  }
}

//----------
//F
//----------

template <typename T, int nX, int nY, int nQm>  
void Kalman<T, nX, nY, nQm>::set_F(T F_in[nX][nX]){
  for (int i = 0; i<nX; i++){
    for (int j = 0; j<nX; j++){
      F[i][j] = F_in[i][j];
    }
  }
}

template <typename T, int nX, int nY, int nQm>  
void Kalman<T, nX, nY, nQm>::set_H(T H_in[nY][nX]){
  for (int i = 0; i<nY; i++){
    for (int j = 0; j<nX; j++){
      H[i][j] = H_in[i][j];
    }
  }
}

template <typename T, int nX, int nY, int nQm>  
void Kalman<T, nX, nY, nQm>::set_dT(T dT_in){
  dT = dT_in;
}

template <typename T, int nX, int nY, int nQm>  
void Kalman<T, nX, nY, nQm>::set_Q(void){
  T c;
  for (int k = 0; k<nX; k++){
    for (int i = k; i<nX; i++){  
      c = 0.0;
      for (int j = 0; j<nQm; j++){
        c += G[k][j]*G[i][j]*Qm[j][0];
      }
      Q[k][i] = c;
      Q[i][k] = c;
    }
  }
}


template <typename T, int nX, int nY, int nQm>  
void Kalman<T, nX, nY, nQm>::set_G(T G_in[nX][nQm]){
  for (int i = 0; i<nX; i++){
    for (int j = 0; j<nQm; j++){
      G[i][j] = G_in[i][j];
    }
  }
}

template <typename T, int nX, int nY, int nQm>  
void Kalman<T, nX, nY, nQm>::set_Y(T Y_in[nY][1]){
  for (int i = 0; i<nY; i++){
    Y[i][0] = Y_in[i][0];
  }
}

template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::get_F(T F_out[nX][nX]){
  for (int i = 0; i<nX; i++){
    for (int j = 0; j<nX; j++){
      F_out[i][j] = F[i][j];
    }
  }
}

template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::get_Q(T Q_out[nX][nX]){
  for (int i = 0; i<nX; i++){
    for (int j = 0; j<nX; j++){
      Q_out[i][j] = Q[i][j];
    }
  }
}

//----------
//Y
//----------



template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::set_Yhat(void){
  m_mul<T, nY, nX,1>(Yhat,H,X); //Yhat = H*X 
}
// 
// 
template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::set_Z(void){
  m_minus<T,nY,1>(Z,Y,Yhat);
}

template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::set_K_seq(int i){
  T Hi[1][nX];
  T Ss[1][1];
  T tmp_1_nX[1][nX];
  m_get_row<T,nY,nX>(Hi,H,i);
  m_mul<T,1,nX,nX>(tmp_1_nX,Hi,P);
  m_mul_t<T,1,nX,1>(Ss,tmp_1_nX,Hi); 
  Ss[0][0] += R[i][0];

  m_mul_t<T,nX,nX,1>(Kseq,P,Hi);
  m_mul_s<T,nX,1>(Kseq,Kseq,1.0/Ss[0][0]);  
}

template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::set_K(void){  
//  S = obj.H*Pkkm*obj.H'+obj.R;
// obj.K = Pkkm*obj.H'*pinv(S);  
  T tmp_nY_nX[nY][nX];
  T tmp_nX_nY[nX][nY];
  m_mul<T,nY,nX,nX>(tmp_nY_nX,H,P);
  m_mul_t<T,nY,nX,nY>(S,tmp_nY_nX,H);
  m_plus_diag<T,nY>(S,R);
  m_inv<T,nY>(iS, S);
  m_mul_t<T,nX,nX,nY>(tmp_nX_nY,P,H);
  m_mul<T,nX,nY,nY>(K,tmp_nX_nY,iS);
}
 

template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::smooth_X_seq(int i){
  //Xkkm = Xkkm+K*z(i);
  T tmp_nX_1[nX][1];
  m_mul_s<T,nX,1>(tmp_nX_1,Kseq,Z[i][0]);
  m_plus<T,nX,1>(X,tmp_nX_1,X);
}

template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::smooth_X(void){
  //Xkkm = Xkkm+K*z(i);
  T tmp_nX_1[nX][1];
  m_mul<T,nX,nY,1>(tmp_nX_1,K,Z);
  m_plus<T,nX,1>(X,tmp_nX_1,X);
}


template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::update(void){
  set_Q();
  predict_P();
  predict_X();
  set_Yhat();
  set_Z();
  #ifndef K_TYPE
  for (int i = 0; i<nY; i++){
    set_K_seq(i);
    smooth_X_seq(i);
    smooth_P_seq(i);
  }
  #else
  set_K();
  smooth_X();
  smooth_P();
  #endif
}
 
template <typename T, int nX, int nY, int nQm>
bool Kalman<T, nX, nY, nQm>::check_P(void){
  for (int i = 0; i<nX; i++){
    for (int j = 0; j<nX; j++){
      if (P[i][j] > 10.0e30){
        return true;
      }
    }
  }
  return false;
}

template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::predict_P(void){
  if (check_P() == false){
    m_mul<T,nX,nX,nX>(M,F,P);
    m_mul_t<T,nX,nX,nX>(P,M,F);
    m_plus<T,nX,nX>(P,P,Q);//P = F*P*F'+Q;
  }
}

template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::smooth_P(void){
//       M = (I-obj.K*obj.H);
// %     obj.P = M*Pkkm;  
//       obj.P = M*Pkkm*M'+obj.K*obj.R*obj.K';
  if (check_P() == false){
    m_mul<T,nX,nY,nX>(M,K,H); //M = K*H
    m_mul_s<T,nX,nX>(M,M,-1.0);//M = -K*Hi
    m_plus_eye<T,nX>(M);
    
    //Pkkm = M*Pkkm*M'+K*obj.R(i,i)*K';
    m_mul<T,nX,nX,nX>(F,M,P);
    m_mul_t<T,nX,nX,nX>(P,F,M); //P = M*P*M'; 
    m_sigma_mul_diag<T,nX,nY>(F,K,R);//tmp13_13 = K*Ri*K'
    m_plus<T,nX,nX>(P,P,F);  
  }
}


template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::smooth_P_seq(int i){
  //M = (I-K*obj.H(i,:)); 
  T Hi[1][nX];
//   T tmp_nX_nX[nX][nX];
  //T Kt[1][nX];
  m_get_row<T,nY,nX>(Hi,H,i); 
  m_mul<T,nX,1,nX>(M,Kseq,Hi); //M = K*Hi
  m_mul_s<T,nX,nX>(M,M,-1.0);//M = -K*Hi
  m_plus_eye<T,nX>(M);

  //Pkkm = M*Pkkm*M'+K*obj.R(i,i)*K';
//   m_tran<T,nX,nX>(Mt,M); 
  m_mul<T,nX,nX,nX>(F,M,P);
  m_mul_t<T,nX,nX,nX>(P,F,M); //P = M*P*M'; 
  
  //m_tran<T,nX,1>(Kt,K); 
  m_mul_t<T,nX,1,nX>(F,Kseq,Kseq);
  m_mul_s<T,nX,nX>(F,F,R[i][0]);//tmp13_13 = K*Ri*K'
  m_plus<T,nX,nX>(P,P,F);
}
//         
template <typename T, int nX, int nY, int nQm>
void Kalman<T, nX, nY, nQm>::predict_X(void){
  T Xk[nX][1];  
  m_mul<T,nX,nX,1>(Xk,F,X);//Xk = F*X
  m_copy<T,nX,1>(X,Xk);
}
  

#endif //KALMAN_HPP
