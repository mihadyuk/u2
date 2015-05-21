#ifndef SINS_HPP
#define SINS_HPP
//#include <math.h>
#include "sins_error_model.hpp"
#include "att.hpp"
#include "matrix_math.hpp"
#include "qmath.hpp"
#include "wgs84.hpp"

template <typename T, int nX>
class Sins{
public:
  Sins(void);
  void set_r(T r_in[3][1]);
  void set_v(T v_in[3][1]);
  void set_qnb(T qnb_in[4][1]);
  void set_dT(T dT_in);
  void get_r(T r_out[3][1]);
  void get_v(T v_out[3][1]);
  void get_qnb(T qnb_out[3][1]);
  void get_eu(T eu_out[3][1]);
  void get_r_sm(T r_out[3][1]);
  void get_v_sm(T v_out[3][1]);
  void get_qnb_sm(T qnb_out[3][1]);
  void get_eu_sm(T eu_sm_out[3][1]);
  void get_free_acc(T free_acc_out[3][1]);
  void update(T fb[3][1], T wb[3][1]);
  void smooth(T X[nX][1]);
  void rst_smooth(bool rst);
  
  void reset_extr(void);

  T phi[3][1];
  T dT;
  T dr_extr[3][1];
  T dv_extr[3][1];
private:  
  T r[3][1];
  T v[3][1];
  T qnb[4][1];
  
  T r_sm[3][1];
  T v_sm[3][1];
  T qnb_sm[4][1];
  
  T dr[3][1];
  T dv[3][1];
  
  T acc_f[3][1];
  T dqnb[4][1];
};

template <typename T, int nX>
Sins<T, nX>::Sins(void){
  qnb[0][0] = 1.0;
  qnb[1][0] = 0.0;
  qnb[2][0] = 0.0;
  qnb[3][0] = 0.0;
  
  qnb_sm[0][0] = 1.0;
  qnb_sm[1][0] = 0.0;
  qnb_sm[2][0] = 0.0;
  qnb_sm[3][0] = 0.0;
  
}


template <typename T, int nX>
void Sins<T, nX>::rst_smooth(bool rst){
  if (rst == true){
    for (int i = 0; i<3; i++){
      r[i][0] = r_sm[i][0];
      v[i][0] = v_sm[i][0];
      qnb[i][0] = qnb_sm[i][0];
    } 
    qnb[3][0] = qnb_sm[3][0];
  }
}

//----------
//r
//----------
template <typename T, int nX>  
void Sins<T, nX>::set_r(T r_in[3][1]){
  for (int i = 0; i<3; i++){
    r[i][0] = r_in[i][0];
    r_sm[i][0] = r_in[i][0];
  }
}

template <typename T, int nX>  
void Sins<T, nX>::set_dT(T dT_in){
  dT = dT_in;
}

template <typename T, int nX>  
void Sins<T, nX>::get_r(T r_out[3][1]){
  for (int i = 0; i<3; i++){
    r_out[i][0] = r[i][0];
  }
}

template <typename T, int nX>  
void Sins<T, nX>::get_r_sm(T r_out[3][1]){
  for (int i = 0; i<3; i++){
    r_out[i][0] = r_sm[i][0];
  }
}

template <typename T, int nX>  
void Sins<T, nX>::set_v(T v_in[3][1]){
  for (int i = 0; i<3; i++){
    v[i][0] = v_in[i][0];
    v_sm[i][0] = v_in[i][0];
  }
}

template <typename T, int nX>  
void Sins<T, nX>::get_v(T v_out[3][1]){
  for (int i = 0; i<3; i++){
    v_out[i][0] = v[i][0];
  }
}

template <typename T, int nX>  
void Sins<T, nX>::get_free_acc(T free_acc_out[3][1]){
  for (int i = 0; i<3; i++){
    free_acc_out[i][0] = acc_f[i][0];
  }
}

template <typename T, int nX>  
void Sins<T, nX>::get_v_sm(T v_out[3][1]){
  for (int i = 0; i<3; i++){
    v_out[i][0] = v_sm[i][0];
  }
}

template <typename T, int nX>  
void Sins<T, nX>::set_qnb(T qnb_in[4][1]){
  for (int i = 0; i<4; i++){
    qnb[i][0] = qnb_in[i][0];
    qnb_sm[i][0] = qnb_in[i][0];
  }
}

template <typename T, int nX>  
void Sins<T, nX>::get_qnb(T qnb_out[4][1]){
  for (int i = 0; i<4; i++){
    qnb_out[i][0] = qnb[i][0];
  }
}

template <typename T, int nX>  
void Sins<T, nX>::get_qnb_sm(T qnb_out[4][1]){
  for (int i = 0; i<4; i++){
    qnb_out[i][0] = qnb_sm[i][0];
  }
}

template <typename T, int nX>  
void Sins<T, nX>::get_eu_sm(T eu_sm_out[3][1]){
  quat2euler<T>(eu_sm_out, qnb_sm);
}

template <typename T, int nX>  
void Sins<T, nX>::get_eu(T eu_out[3][1]){
  quat2euler<T>(eu_out, qnb);
}

template <typename T, int nX>
void Sins<T, nX>::update(T fb[3][1], T Wbib[3][1]){
  T Cnb[3][3], Cbn[3][3], Wcec[3][1], Wcie[3][1], Wcic[3][1], 
    Wcie_skew[3][3],Wcic_skew[3][3], Wcie_p_Wcic_skew[3][3];
  T Ge[3][1];
  T fn[3][1];
  
  //set Ge
  get_Ge<T>(Ge, r);
  
  //ideal
  // Ge[0][0] = 0.0f;
  // Ge[1][0] = 0.0f;
  // Ge[2][0] = 9.81f;
  //
  quat2dcm(Cnb,qnb);
  m_tran<T,3,3>(Cbn,Cnb);
  
  m_set<T,3,1>(dr,0.0);
  m_set<T,3,1>(dv,0.0);
  
  //set fn
  m_mul<T,3,3,1>(fn,Cnb,fb);
  
  //set cross
  get_WcecWcie<T>(Wcec, Wcie, r, v);
  
  m_plus<T,3,1>(Wcic,Wcec,Wcie);
  cpm<T>(Wcie_skew, Wcie);
  cpm<T>(Wcic_skew, Wcic);
  m_plus<T,3,3>(Wcie_p_Wcic_skew,Wcie_skew,Wcic_skew);
  
  //set dv
  m_mul<T,3,3,1>(dv, Wcie_p_Wcic_skew, v);
  m_mul_s<T,3,1>(dv, dv, -1.0);
  m_plus<T,3,1>(dv, dv, fn);
  m_plus<T,3,1>(dv, dv, Ge);
  m_copy<T,3,1>(acc_f,dv);
  m_mul_s<T,3,1>(dv, dv, dT);
  
  //set dr
  T Di[3][3];
  get_Di(Di,r);
  m_mul<T,3,3,1>(dr, Di, v);
  m_mul_s<T,3,1>(dr, dr, dT);
    
  //set dq
  T Wbin[3][1], Wbnb[3][1], qw[4][1];
  m_mul<T,3,3,1>(Wbin,Cbn,Wcic);
  m_minus<T,3,1>(Wbnb, Wbib, Wbin);
  
  vec2quat<T>(qw, Wbnb);
  qmult<T>(dqnb, qnb, qw);
  m_mul_s<T,4,1>(dqnb, dqnb, 0.5*dT);
  
  //Update
  m_plus<T,3,1>(r, r, dr);
  m_plus<T,3,1>(v, v, dv);
  m_plus<T,4,1>(qnb, qnb, dqnb);
  qnorm<T>(qnb);
  
  m_plus<T,3,1>(dr_extr, dr_extr, dr);
  m_plus<T,3,1>(dv_extr, dv_extr, dv);
}

template <typename T, int nX>  
void Sins<T, nX>::reset_extr(void){
  m_set<T,3,1>(dr_extr,0.0);
  m_set<T,3,1>(dv_extr,0.0);  
}

template <typename T, int nX>  
void Sins<T, nX>::smooth(T X[nX][1]){
  
  T dr[3][1];
  T dvx[3][1];
  T dv[3][1];
  T dth[3][1];
  T dth_skew[3][3];
  T psi[3][1];
  //T phi[3][1];
  T tmp[3][1];
  get_sub<T, 3, 1, nX, 1>(dr, X, 0, 0);
  get_sub<T, 3, 1, nX, 1>(dvx, X, 3, 0);
  get_sub<T, 3, 1, nX, 1>(psi, X, 6, 0);
  get_dth<T>(dth, r, dr);
  cpm<T>(dth_skew, dth);
  
  m_plus<T,3,1>(phi,psi,dth);  
  
  m_mul<T,3,3,1>(tmp,dth_skew,v);
  m_minus<T,3,1>(dv,dvx,tmp);
  m_minus<T,3,1>(v_sm,v,dv);
  
  //r corr
  T qec[4][1], qcn[4][1], mdth[3][1], qen[4][1];
  m_mul_s<T,3,1>(mdth,dth,-1.0);
  pos2quat(qec, r);
  rot2quat(qcn, mdth);
  qmult(qen, qec, qcn);
  qnorm(qen);
  quat2pos(r_sm,qen); 
  r_sm[2][0] = r[2][0]+dr[2][0];
  
  //q corr
  T qcor[4][1];
  rot2quat<T>(qcor, phi);
  qmult(qnb_sm, qcor, qnb);
  qnorm(qnb_sm);
}

#endif //SINS_HPP
