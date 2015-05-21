#ifndef ALIGNMENT_HPP
#define ALIGNMENT_HPP
#include "kalman.hpp"
#include "matrix_math.hpp"

template <typename T, int nY>
class Alignment{
public:
  Alignment(void);
  void setY(T eu_nb[3][1], int i);
  void setYhat(T eu_nh[3][1], int i);
  void setZ(void);
  void update(void);
  
  T qhb[4][1];  
  T eu_hb[3][1];
  int iter;
    
  private:
  T Y[nY][1];
  T Yhat[nY][1];
  T Z[nY][1];
};

template <typename T, int nY>
Alignment<T, nY>::Alignment(void)
{  
  qhb[0][0] = 1.0;
  for (int i = 1; i<4; i++){
    qhb[i][0] = 0.0;
    eu_hb[i-1][0] = 0.0;
  }
  iter = 10;
}

template <typename T, int nY>
void Alignment<T, nY>::update(void)
{
  for(int i = 0; i<iter; i++){
    
  }
}

// template <typename T, int nY>
// Alignment<T, nY>::setY(T eu_nb[3][1], int i)
// {  
//   Y[2*i][0] = eu_nb[0][0];
//   Y[2*i+1][0] = eu_nb[1][0];
// }
// 
// template <typename T, int nY>
// Alignment<T, nY>::setYhat(T eu_nh[3][1], int i)
// {  
//    T qnb_hat[4][1];
//    T qnh[4][1];
//    T eu_nb_hat[3][1];
//    
//    euler2quat<T>(qnh, eu_nh);
//    qmul<T>(qnb_hat, qnh, qhb);
//    quat2euler<T>(eu_nb_hat, qnb_hat);
//    Yhat[2*i][0] = eu_nb_hat[0][0];  
//    Yhat[2*i+1][0] = eu_nb_hat[1][0];
// }
// 
// template <typename T, int nY>
// Alignment<T, nY>::setZ(void)
// {  
//    m_minus<T,nY,1>(Z,Y,Yhat);
// }
// 
// // template <typename T>
// // bool Sins_Kalman_LC<T, nX, nY>::initialize(T r_init[3][1], T v_init[3][1], T qnb_init[4][1],
// //                                            T R[9][1], T Qm[8][1], T P[10][1], T dT_in){
// //   S.set_r(r_init);
// //   S.set_v(v_init);
// //   S.set_qnb(qnb_init); 
// //   S.set_dT(dT_in); 
// //     
// //   /* initialize R matrix.*/
// //   m_set<T,nY,1>(Filter.R,0.0);
// //   
// //   //sns r(1-2) (N and E)
// //   Filter.R[0][0] = R[0][0]*R[0][0];
// //   Filter.R[1][0] = R[0][0]*R[0][0];
// //   
// //   //sns r(1-3) (D)
// //   Filter.R[2][0] = R[1][0]*R[1][0];
// // 
// //   //sns velocity
// //   Filter.R[3][0] = R[2][0]*R[2][0];
// //   Filter.R[4][0] = R[2][0]*R[2][0];
// //   Filter.R[5][0] = R[2][0]*R[2][0];
// //    
// //   //odo
// //   Filter.R[6][0] = R[3][0]*R[3][0];
// //     
// //   //non_hol
// //   Filter.R[7][0] = R[4][0]*R[4][0];
// //   Filter.R[8][0] = R[4][0]*R[4][0];
// //   
// //   //alt_baro
// //   Filter.R[9][0] = R[5][0]*R[5][0];
// //   
// //   //mag
// //   Filter.R[10][0] = R[6][0]*R[6][0];
// //   Filter.R[11][0] = R[6][0]*R[6][0];
// //   Filter.R[12][0] = R[6][0]*R[6][0];
// //   
// //   //ang
// //   Filter.R[13][0] = R[7][0]*R[7][0];
// //   Filter.R[14][0] = R[7][0]*R[7][0];
// //   Filter.R[15][0] = R[7][0]*R[7][0];
// //   
// //   //ZIHR
// //   Filter.R[16][0] = R[8][0]*R[8][0];
// //   
// //   //
// //   /* initialize P matrix.*/
// //   m_set<T,nX,nX>(Filter.P,0.0);
// //   set_diag_cov<T,nX>(Filter.P, 0, 2, P[0][0]);
// //   set_diag_cov<T,nX>(Filter.P, 3, 5, P[1][0]);
// //   set_diag_cov<T,nX>(Filter.P, 6, 7, P[2][0]);
// //   set_diag_cov<T,nX>(Filter.P, 8, 8, P[3][0]);
// //   if (nX > 9){
// //     set_diag_cov<T,nX>(Filter.P, 9, 11, P[4][0]);
// //     set_diag_cov<T,nX>(Filter.P, 12, 14, P[5][0]);
// //   }
// //   if (nX > 15){
// //     set_diag_cov<T,nX>(Filter.P, 15, 17, P[6][0]);
// //     set_diag_cov<T,nX>(Filter.P, 18, 20, P[7][0]);
// //   }
// //   if (nX > 21){
// //     set_diag_cov<T,nX>(Filter.P, 21, 26, P[8][0]);
// //     set_diag_cov<T,nX>(Filter.P, 27, 32, P[9][0]);
// //   }
// //   
// //   /* initialize Qm matrix.*/
// //   m_set<T,nX-3,1>(Filter.Qm,0.0);
// //   
// //   for(int i = 0; i<=2; i++){
// //     Filter.Qm[i][0] = Qm[0][0]*Qm[0][0];
// //   }
// //   for(int i = 3; i<=5; i++){
// //     Filter.Qm[i][0] = Qm[1][0]*Qm[1][0];
// //   }
// //   
// //   if (nX > 9){
// //     for(int i = 6; i<=8; i++){
// //       Filter.Qm[i][0] = Qm[2][0]*Qm[2][0];
// //     }
// //     for(int i = 9; i<=11; i++){
// //       Filter.Qm[i][0] = Qm[3][0]*Qm[3][0];
// //     }
// //   }
// //   
// //   if (nX > 15){   
// //     for(int i = 12; i<=14; i++){
// //       Filter.Qm[i][0] = Qm[4][0]*Qm[4][0];
// //     }
// //     for(int i = 15; i<=17; i++){
// //       Filter.Qm[i][0] = Qm[5][0]*Qm[5][0];
// //     }
// //   }
// //   if (nX > 21){
// //     for(int i = 18; i<=23; i++){
// //       Filter.Qm[i][0] = Qm[6][0]*Qm[6][0];
// //     }
// //     for(int i = 24; i<=29; i++){
// //       Filter.Qm[i][0] = Qm[7][0]*Qm[7][0];
// //     }
// //   }
// // 
// //   
// //   Filter.rst(true);
// //   Filter.set_dT(dT_in);
// //   course_prev[0][0] = 0.0;
// //   initialized = true;
// //   S.get_r_sm(r_sm);
// //   S.get_v_sm(v_sm);
// //   S.get_qnb_sm(qnb_sm);
// //   S.get_eu_sm(eu_sm);
// //   return true;
// // }
// // 
// // template <typename T, int nX, int nY>
// // void Sins_Kalman_LC<T, nX, nY>::set_qhv(bool rst_yaw){
// //   T qnh[4][1];
// //   T qhn[4][1];
// //   T qbh[4][1];
// //   qcon<T>(qbh, qhb);
// //   
// //   qmult<T>(qnh, qnb_sm, qbh);
// //   qnorm<T>(qnh);
// //   
// //   if (rst_yaw == true){
// //     T eu_int[3][1];
// //     quat2euler<T>(eu_int, qnh);
// //     eu_int[2][0] = 0.0;
// //     euler2quat<T>(qnh, eu_int);
// //   } 
// //   qcon<T>(qhn, qnh);
// //   
// // //    //qhv = qhb*qbn*qnv
// // //    T qbn[4][1];
// // //    T qbv[4][1];
// // //    qcon<T>(qbn, qnb_int);
// // //    qmult<T>(qbv, qbn, qnv_init_base);
// // //    qnorm<T>(qbv);
// // //    
// //    if (active_platform == VEH_BASE){
// //      qmult<T>(qhv_base, qhn, qnv_init_base);
// //      qnorm<T>(qhv_base);
// //    }
// //    else{
// //      qmult<T>(qhv_turret, qhn, qnv_init_turret);
// //      qnorm<T>(qhv_turret);
// //    }
// // }
// // 
// // template <typename T, int nX, int nY>
// // void Sins_Kalman_LC<T, nX, nY>::set_qhb(T eu_hb[3][1]){
// //   euler2quat<T>(qhb, eu_hb);
// // }
// // 
// // template <typename T, int nX, int nY>
// // void Sins_Kalman_LC<T, nX, nY>::set_qnv_init_base(T eu_nv_init[3][1]){
// //   euler2quat<T>(qnv_init_base, eu_nv_init);
// // }
// // 
// // template <typename T, int nX, int nY>
// // void Sins_Kalman_LC<T, nX, nY>::set_qnv_init_turret(T eu_nv_init[3][1]){
// //   euler2quat<T>(qnv_init_turret, eu_nv_init);
// // }
// // 
// // 
// // 
// // template <typename T, int nX, int nY>
// // void Sins_Kalman_LC<T, nX, nY>::update(T r_sns[3][1], T v_sns[3][1], T alt_b[1][1], T v_odo[3][1], T mb[3][1], T ang[3][1],
// //                                T fb[3][1], T wb[3][1], T mn[3][1], bool rst, KalmanFlags en_flg){
// //     
// //   // Corrector
// //   T r[3][1];
// //   T v[3][1];
// //   T qnb[4][1];
// //   T eu[3][1];
// //     
// //   T r_sm_int[3][1];
// //   T v_sm_int[3][1];
// //   T qnb_sm_int[4][1];
// //   T eu_sm_int[3][1];
// // 
// //   T qbv[4][1];
// //   
// //   Calib.calibration(fb_c, wb_c, fb, wb); 
// //   
// //   //SINS 
// //   S.update(fb_c, wb_c);
// //   
// //   S.get_r(r);
// //   S.get_v(v);
// //   S.get_qnb(qnb); 
// //   
// //   S.get_r_sm(r_sm_int);
// //   S.get_v_sm(v_sm_int);
// //   S.get_qnb_sm(qnb_sm_int);
// //   S.get_eu_sm(eu_sm_int);
// //     
// //   //Kalman
// //   set_FG_psi<T, nX, nX-3>(Filter.F, Filter.G, r_sm_int, v_sm_int, qnb_sm_int, fb_c, wb_c, Filter.dT);
// //      
// //   //set H
// //   //qbv = qbh*qhv;
// //   
// //   T qbh[4][1];
// //   qcon<T>(qbh, qhb);
// //   qmult<T>(qbv, qbh, qhv_base);
// //   qnorm<T>(qbv);     
// // 
// //   m_set<T,nY,nX>(Filter.H,0.0);
// //   set_H<T, nY, nX>(Filter.H, en_flg, qnb_sm_int, qbv, r_sm_int, v_sm_int, mn, wb_c, S.phi, l_sns, l_wheel, S.dT);
// //   
// //   m_set<T,nY,1>(Filter.Y,0.0);
// //   
// //   T alt_fix[1][1];
// //   alt_fix[0][0] = alt_b[0][0];
// //   if (en_flg.baro_fix_en == true){
// //     alt_fix[0][0] += dwgs_baro[0][0];
// //   }
// // 
// //   T qnv[4][1];
// //   qmult<T>(qnv, qnb, qhv_base);
// //   qnorm<T>(qnv);
// //   
// //   quat2euler<T>(eu, qnv);
// //   
// //   //measurement
// //   set_Y<T, nY>(Filter.Y, r, v, r_sns, v_sns, alt_fix, v_odo, mb, mn, ang, course_prev, qnb_sm_int, qbv, eu);
// //   
// //   Filter.update();  
// //   
// //   //bias integrator update
// //   Calib.update(Filter.X, rst);
// //   
// //   //smooth
// //   S.smooth(Filter.X); 
// //   
// //   //reset
// //   S.rst_smooth(rst);
// //   Filter.rst(rst);
// //   
// //   //output
// //   S.get_r_sm(r_sm);
// //   S.get_v_sm(v_sm);
// //   S.get_qnb_sm(qnb_sm);
// //   S.get_eu_sm(eu_sm);
// //   
// //   
// //   //set qvn
// //   T qbv_base[4][1];
// //   T qbv_turret[4][1];
// //   qmult<T>(qbv_base, qbh, qhv_base);
// //   qnorm<T>(qbv_base);
// //   qmult<T>(qbv_turret, qbh, qhv_turret);
// //   qnorm<T>(qbv_turret);
// //   
// // //qmult<T>(qnv, qnb, qhv_base);
// // //qnorm<T>(qnv);
// //   
// //   qmult<T>(qnv_base, qnb_sm, qbv_base);
// //   qnorm<T>(qnv_base);
// //   quat2euler<T>(eu_nv_base, qnv_base);
// // 
// //   qmult<T>(qnv_turret, qnb_sm, qbv_turret);
// //   qnorm<T>(qnv_turret);
// //   quat2euler<T>(eu_nv_turret, qnv_turret);
// // 
// // 
// //   if (active_platform == VEH_BASE){
// //     m_copy<T,4,1>(qnv_out,qnv_base);
// //     m_copy<T,3,1>(eu_nv_out,eu_nv_base);
// //   }
// //   else
// //   {
// //     m_copy<T,4,1>(qnv_out,qnv_turret);
// //     m_copy<T,3,1>(eu_nv_out,eu_nv_turret);
// //   }
// //   ///
// //   
// //   
// //   S.get_free_acc(acc_f);
// //   Calib.get_bias(a_bias, w_bias);
// //   Calib.get_scale(a_scale, w_scale);
// //   //ZIHR
// //   course_prev[0][0] = eu_nv_out[2][0];
// //    
// //   //delta wgs84 and baroheight
// //   if (en_flg.sns_r_en == true){
// //     dwgs_baro[0][0] = (1.0-alpha)*dwgs_baro[0][0]+alpha*(r_sns[2][0]-alt_b[0][0]);
// //   }
// // }
#endif //ALIGNMENT_HPP
