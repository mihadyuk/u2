#ifndef SINS_KALMAN_LC_HPP
#define SINS_KALMAN_LC_HPP
#include "imu_calibrator.hpp"
#include "kalman.hpp"
#include "sins.hpp"
#include "sins_error_model.hpp"

#define DWGS_ALT_ALPHA_DEFAULT 0.1f

typedef enum {
  VEH_BASE  = 0,
  VEH_TURRET = 1
}vehicle_mode_t;

template <typename T, int nX, int nY>
class Sins_Kalman_LC{
public:
  Sins_Kalman_LC(void);
  bool initialize(T r_init[3][1], T v_init[3][1], T qnv_init[4][1],
                  T R[9][1], T Qm[10][1], T P[10][1], T dT_in, unsigned int rst_cnt_max);
  void set_qnb_init(T qnv_init[4][1]);
  void init_nav(T r_init[3][1], T v_init[3][1], T qnb_init[4][1]);
  void init_time_const(T dT_in, unsigned int rst_cnt_max);
  void init_RQmP(T R[9][1], T Qm[10][1], T P[10][1]);
  
  void set_RQ(T R[9][1], T Qm[10][1]);
  void set_P(T P[10][1]);

  void update(T r_sns[3][1], T v_sns[3][1], T alt_b[1][1], T v_odo[3][1], T mb[3][1], T ang[3][1],
              T fb[3][1], T wb[3][1], T mn[3][1], KalmanFlags en_flg);
 
  void rst_control(void);
  void set_qvh_base(T eu_vh_base[3][1]);
  void set_qvh_turret(T eu_vh_turret[3][1]);
  void set_qhb(T eu_hb[3][1]);
  void set_dT(T dT_in);
  void set_qnv(void);
  void set_lever_arm(T l_arm_sns[3][1], T l_arm_wheel[3][1]);
  vehicle_mode_t active_platform;
  
  T qhb[4][1]; //body to housing rotation
  T qvh_base[4][1];
  T qvh_turret[4][1];
  T qbv[4][1];
  T qnb_init_int[4][1];
  
  T r_sm[3][1];
  T v_sm[3][1];
  T qnb_sm[4][1];
  T eu_sm[3][1];
  
  void get_nav(void);
  void get_bias_scale(void);
  //pure sins
  T r_sins[3][1];
  T v_sins[3][1];
  T eu_sins[3][1];
  // vehicle frame output
  T qnv_base[4][1];
  T eu_nv_base[3][1];

  T qnv_turret[4][1];
  T eu_nv_turret[3][1];
  
  //
  T qnv_out[4][1];
  T eu_nv_out[3][1];

  T a_bias[3][1];
  T w_bias[3][1];
  
  T a_scale[3][3];
  T w_scale[3][3];

  T a_no[6][1];
  T w_no[6][1];
  
  T fb_c[3][1];
  T wb_c[3][1];
  T acc_f[3][1];
  T course_prev[1][1];
  T dwgs_baro[1][1];
  T alpha;
  T l_sns[3][1];
  T l_wheel[3][1];
  bool rst;
  Imu_calibrator<T, nX> Calib;

  Sins<T,nX> S;
private:
  Kalman<T,nX,nY,nX-3> Filter;
  bool initialized;
  unsigned int rst_cnt;
  unsigned int rst_max;
};

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::set_lever_arm(T l_arm_sns[3][1], T l_arm_wheel[3][1]){
  for (int i = 0; i<3; i++){
    l_sns[i][0] = l_arm_sns[i][0];
    l_wheel[i][0] = l_arm_wheel[i][0];
  }
}


template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::rst_control(void){
  if (rst == true){
    rst = false;
  }
  rst_cnt += 1;
  if (rst_cnt == rst_max){
    rst_cnt = 0;
    rst = true;
  }
}

template <typename T, int nX, int nY>
Sins_Kalman_LC<T, nX, nY>::Sins_Kalman_LC(void):
initialized(false)
{  
  active_platform = VEH_BASE;
  qvh_base[0][0] = 1.0;
  qvh_turret[0][0] = 1.0;
  qnv_base[0][0] = 1.0;
  qnv_turret[0][0] = 1.0;
  qnv_out[0][0] = 1.0;
  qhb[0][0] = 1.0;
  qbv[0][0] = 1.0;
  rst_cnt = 0;
  rst = false;
  rst_max = 35; //~1.5 sec if dT = 0.04 sec
  set_dT(0.04f);
  for (int i = 1; i<4; i++){
    qbv[i][0] = 0.0;
    qhb[i][0] = 0.0;
    qvh_base[i][0] = 0.0;
    qvh_turret[i][0] = 0.0;
    qnv_base[i][0] = 0.0;
    qnv_turret[i][0] = 0.0;
    qnv_out[i][0] = 0.0;
    l_sns[i-1][0] = 0.0;
    l_wheel[i-1][0] = 0.0;
  }
  
  dwgs_baro[0][0] = 0.0;
  alpha = DWGS_ALT_ALPHA_DEFAULT;
  
  m_set<T,nY,1>(Filter.R,1.0);
  m_set<T,nX-3,1>(Filter.Qm,0.0);
  m_set<T,nX,nX>(Filter.P,0.0);
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::set_RQ(T R[9][1], T Qm[10][1]){
/* initialize R matrix.*/
  m_set<T,nY,1>(Filter.R,0.0);
  
  //sns r(1-2) (N and E)
  Filter.R[0][0] = R[0][0]*R[0][0];
  Filter.R[1][0] = R[0][0]*R[0][0];
  
  //sns r(1-3) (D)
  Filter.R[2][0] = R[1][0]*R[1][0];

  //sns velocity
  Filter.R[3][0] = R[2][0]*R[2][0];
  Filter.R[4][0] = R[2][0]*R[2][0];
  Filter.R[5][0] = R[2][0]*R[2][0];
   
  if (nY > 6){ 
    //odo
    Filter.R[6][0] = R[3][0]*R[3][0];
    //non_hol
    Filter.R[7][0] = R[4][0]*R[4][0];
    Filter.R[8][0] = R[4][0]*R[4][0];
  }
  
  if (nY > 9){
    //alt_baro
    Filter.R[9][0] = R[5][0]*R[5][0];

    //mag
    Filter.R[10][0] = R[6][0]*R[6][0];
    Filter.R[11][0] = R[6][0]*R[6][0];
    Filter.R[12][0] = R[6][0]*R[6][0];
  
    //ang
    Filter.R[13][0] = R[7][0]*R[7][0];
    Filter.R[14][0] = R[7][0]*R[7][0];
    Filter.R[15][0] = R[7][0]*R[7][0];
  
    //ZIHR
    Filter.R[16][0] = R[8][0]*R[8][0];
  }
  //
  
  /* initialize Qm matrix.*/
  m_set<T,nX-3,1>(Filter.Qm,0.0);
  for(int i = 0; i<=2; i++){ //acc
    Filter.Qm[i][0] = Qm[0][0]*Qm[0][0];
  }
  for(int i = 3; i<=5; i++){ //gyro
    Filter.Qm[i][0] = Qm[1][0]*Qm[1][0];
  }
  
  if (nX > 9){
    Filter.Qm[6][0] = Qm[2][0]*Qm[2][0];
    Filter.Qm[7][0] = Qm[3][0]*Qm[3][0];
    Filter.Qm[8][0] = Qm[4][0]*Qm[4][0];
    for(int i = 9; i<=11; i++){  //gyro bias
      Filter.Qm[i][0] = Qm[5][0]*Qm[5][0];
    }
  }
    if (nX > 15){   
    for(int i = 12; i<=14; i++){ //acc scale
      Filter.Qm[i][0] = Qm[6][0]*Qm[6][0];
    }
    for(int i = 15; i<=17; i++){ //gyro scale
      Filter.Qm[i][0] = Qm[7][0]*Qm[7][0];
    }
  }
  if (nX > 21){
    for(int i = 18; i<=23; i++){ //acc nonort
      Filter.Qm[i][0] = Qm[8][0]*Qm[8][0];
    }
    for(int i = 24; i<=29; i++){ //gyro nonort
      Filter.Qm[i][0] = Qm[9][0]*Qm[9][0];
    }
  }
};


template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::set_P(T P[10][1]){
  /* initialize P matrix.*/
  m_set<T,nX,nX>(Filter.P,0.0);
  set_diag_cov<T,nX>(Filter.P, 0, 2, P[0][0]);
  set_diag_cov<T,nX>(Filter.P, 3, 5, P[1][0]);
  set_diag_cov<T,nX>(Filter.P, 6, 7, P[2][0]);
  set_diag_cov<T,nX>(Filter.P, 8, 8, P[3][0]);
  if (nX > 9){
    set_diag_cov<T,nX>(Filter.P, 9, 11, P[4][0]);
    set_diag_cov<T,nX>(Filter.P, 12, 14, P[5][0]);
  }
  if (nX > 15){
    set_diag_cov<T,nX>(Filter.P, 15, 17, P[6][0]);
    set_diag_cov<T,nX>(Filter.P, 18, 20, P[7][0]);
  }
  if (nX > 21){
    set_diag_cov<T,nX>(Filter.P, 21, 26, P[8][0]);
    set_diag_cov<T,nX>(Filter.P, 27, 32, P[9][0]);
  }
};

template <typename T, int nX, int nY>
bool Sins_Kalman_LC<T, nX, nY>::initialize(T r_init[3][1], T v_init[3][1], T qnv_init[4][1],
                                           T R[9][1], T Qm[10][1], T P[10][1], T dT_in, unsigned int rst_cnt_max){
  set_qnb_init(qnv_init);
  init_nav(r_init, v_init, this->qnb_init_int);
  init_time_const(dT_in, rst_cnt_max);
  init_RQmP(R, Qm, P);
   
  get_nav();
  get_bias_scale();
  
  //pure sins set
  S.get_r(r_sins);
  S.get_v(v_sins);
  S.get_eu(eu_sins);
  initialized = true;
  return true;
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::get_nav(void){
  S.get_r_sm(r_sm);
  S.get_v_sm(v_sm);
  S.get_qnb_sm(qnb_sm);
  S.get_eu_sm(eu_sm);  
  S.get_free_acc(acc_f);
  set_qnv();
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::get_bias_scale(void){
  Calib.get_bias(a_bias, w_bias);
  Calib.get_scale(a_scale, w_scale);  
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::init_nav(T r_init[3][1], T v_init[3][1], T qnb_init[4][1]){
  S.set_r(r_init);
  S.set_v(v_init);
  S.set_qnb(qnb_init);
  this->get_nav();
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::init_time_const(T dT_in, unsigned int rst_cnt_max){
  S.set_dT(dT_in);
  rst_max = rst_cnt_max;
  //Filter.rst(true);
  Filter.set_dT(dT_in);
  //course_prev[0][0] = 0.0f;
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::init_RQmP(T R[9][1], T Qm[10][1], T P[10][1]){
  set_RQ(R, Qm);
  set_P(P);
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::set_qhb(T eu_hb[3][1]){
  euler2quat<T>(qhb, eu_hb);
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::set_qvh_base(T eu_vh_base[3][1]){
  euler2quat<T>(qvh_base, eu_vh_base);
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::set_qvh_turret(T eu_vh_turret[3][1]){
  euler2quat<T>(qvh_turret, eu_vh_turret);
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::update(T r_sns[3][1], T v_sns[3][1], T alt_b[1][1], T v_odo[3][1], T mb[3][1], T ang[3][1],
                                       T fb[3][1], T wb[3][1], T mn[3][1], KalmanFlags en_flg){
    
  // Corrector
  T r[3][1];
  T v[3][1];
  T qnb[4][1];
  T eu_nv[3][1];
    
  T r_sm_int[3][1];
  T v_sm_int[3][1];
  T qnb_sm_int[4][1];
  T eu_sm_int[3][1];
  
  rst_control();
  Calib.calibration(fb_c, wb_c, fb, wb); 
  
  //SINS 
  S.update(fb_c, wb_c);
  
  S.get_r(r);
  S.get_v(v);
  S.get_qnb(qnb); 
  
  //pure sins set
//   S.get_r(r_sins);
//   S.get_v(v_sins);
//   S.get_eu(eu_sins);
  
  //smooth
  S.get_r_sm(r_sm_int);
  S.get_v_sm(v_sm_int);
  S.get_qnb_sm(qnb_sm_int);
  S.get_eu_sm(eu_sm_int);
    
  //Kalman
  set_FG_psi<T, nX, nX-3>(Filter.F, Filter.G, r_sm_int, v_sm_int, qnb_sm_int, fb_c, wb_c, Filter.dT);
     
  //set H
  //qbv = qbh*qhv;
  
  T qbh[4][1];
  T qhv_base[4][1];
  T qhv_turret[4][1];
  qcon<T>(qbh, qhb);
  qcon<T>(qhv_base, qvh_base);
  qcon<T>(qhv_turret, qvh_turret);
    
  qmult<T>(qbv, qbh, qhv_base);
  qnorm<T>(qbv);     
  

  m_set<T,nY,nX>(Filter.H,0.0);
  set_H<T, nY, nX>(Filter.H, en_flg, qnb_sm_int, qbv, r_sm_int, v_sm_int, mn, wb_c, S.phi, l_sns, l_wheel, S.dT);
  
  m_set<T,nY,1>(Filter.Y,0.0);
  
  T alt_fix[1][1];
  alt_fix[0][0] = alt_b[0][0];
  if (en_flg.baro_fix_en == true){
    alt_fix[0][0] += dwgs_baro[0][0];
  }

  T qnv[4][1];
  qmult<T>(qnv, qnb, qhv_base);
  qnorm<T>(qnv);
  
  quat2euler<T>(eu_nv, qnv);
  
  //measurement
  set_Y<T, nY>(Filter.Y, r, v, r_sns, v_sns, alt_fix, v_odo, mb, mn, ang, course_prev, qnb_sm_int, qbv, eu_nv, l_sns, wb);
  
  Filter.update();  
  
  //bias integrator update
  Calib.update(Filter.X, rst);
  
  //smooth
  S.smooth(Filter.X); 
  
  //reset
  S.rst_smooth(rst);
  Filter.rst(rst);
  
  //output from SINS
  get_nav();
  get_bias_scale();
  
  //ZIHR
  course_prev[0][0] = eu_nv_out[2][0];
   
  //delta wgs84 and baroheight
  if (en_flg.sns_r_en == true){
    dwgs_baro[0][0] = (1.0-alpha)*dwgs_baro[0][0]+alpha*(r_sns[2][0]-alt_b[0][0]);
  }
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::set_qnv(void){
  //set qnv
  T qbv_base[4][1];
  T qbv_turret[4][1];
  T qbh[4][1];
  T qhv_base[4][1];
  T qhv_turret[4][1];
  qcon<T>(qbh, qhb);
  qcon<T>(qhv_base, qvh_base);
  qcon<T>(qhv_turret, qvh_turret);
    
  qmult<T>(qbv, qbh, qhv_base); 
  qmult<T>(qbv_base, qbh, qhv_base);
  qnorm<T>(qbv_base);
  qmult<T>(qbv_turret, qbh, qhv_turret);
  qnorm<T>(qbv_turret);
  
  qmult<T>(qnv_base, qnb_sm, qbv_base);
  qnorm<T>(qnv_base);
  quat2euler<T>(eu_nv_base, qnv_base);

  qmult<T>(qnv_turret, qnb_sm, qbv_turret);
  qnorm<T>(qnv_turret);
  quat2euler<T>(eu_nv_turret, qnv_turret);

  if (active_platform == VEH_BASE){
    m_copy<T,4,1>(qnv_out,qnv_base);
    m_copy<T,3,1>(eu_nv_out,eu_nv_base);
  }
  else
  {
    m_copy<T,4,1>(qnv_out,qnv_turret);
    m_copy<T,3,1>(eu_nv_out,eu_nv_turret);
  }
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::set_qnb_init(T qnv_init[4][1]){
  //set qnb_init
  //T qbv_base[4][1];
  T qvb_base[4][1];
  T qvb_turret[4][1];
  qmult<T>(qvb_base, this->qvh_base, this->qhb);
  qnorm<T>(qvb_base);
  qmult<T>(qvb_base, this->qvh_turret, this->qhb);
  qnorm<T>(qvb_base);
 
  if (active_platform == VEH_BASE){
    qmult<T>(this->qnb_init_int, qnv_init, qvb_base);
  }
  else
  {
	  qmult<T>(this->qnb_init_int, qnv_init, qvb_turret);
  }
  qnorm<T>(this->qnb_init_int);
}

template <typename T, int nX, int nY>
void Sins_Kalman_LC<T, nX, nY>::set_dT(T dT_in){
  S.set_dT(dT_in);
  Filter.set_dT(dT_in);
}

#endif //SINS_KALMAN_LC_HPP
