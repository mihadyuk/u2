#ifndef IMU_CALIBRATOR_HPP
#define IMU_CALIBRATOR_HPP
#include "matrix_math.hpp"
#include <math.h>

#define PI_M 3.1415926535897932384626433832795028841971693993751f
#define DEG2RAD PI_M/180.0
#define RAD2DEG 180.0/PI_M
#define ALPHA_DEFAULT 0.05

#define BA_SAT_DEFAULT 100000.0
#define SA_SAT_DEFAULT 100000.0
#define BW_SAT_DEFAULT 100000.0*DEG2RAD
#define SW_SAT_DEFAULT 100000.0
#define GA_SAT_DEFAULT 1.0
#define GW_SAT_DEFAULT 1.0

  
template <typename T, int nX>
class Imu_calibrator{
public:
  Imu_calibrator(void);
  void update(T X[nX][1], bool upd_en);
  void gyro_bias_est(T w_in[3][1], bool set);
  
  void get_bias(T ba_out[3][1], T bw_out[3][1]);
  void get_scale(T am_out[3][3], T wm_out[3][3]);
  void calibration(T a_out[3][1], T w_out[3][1], T a_in[3][1], T w_in[3][1]);
  void set_ba_sat(T ba_sat_in[3][1]);
  void set_sa_sat(T sa_sat_in[3][1]);
  void set_bw_sat(T bw_sat_in[3][1]);
  void set_sw_sat(T sw_sat_in[3][1]);
  void set_ga_sat(T ga_sat_in);
  void set_gw_sat(T gw_sat_in);
  void set_alpha(T alpha_in);
  
private:  
  T alpha;
  T beta;
  
  T ba[3][1];
  T sa[3][1];
  T ga[6][1];
  
  T bw[3][1];
  T sw[3][1];
  T gw[6][1];
  
  T ba_sat[3][1];
  T sa_sat[3][1];
  T ga_sat[1][1];
    
  T bw_sat[3][1];
  T sw_sat[3][1];
  T gw_sat[1][1];
  void saturation(void);
};

template <typename T, int nX>
Imu_calibrator<T, nX>::Imu_calibrator(void){
  
  alpha = ALPHA_DEFAULT;
  beta = 1.0-alpha;
  for (int i = 0; i<=2; i++){
    ba[i][0] = 0.0;
    sa[i][0] = 0.0;
    
    bw[i][0] = 0.0;
    sw[i][0] = 0.0;
    
    ba_sat[i][0] = BA_SAT_DEFAULT;
    sa_sat[i][0] = SA_SAT_DEFAULT;
    
    bw_sat[i][0] = BW_SAT_DEFAULT;
    sw_sat[i][0] = SW_SAT_DEFAULT;
  }
  
  ga_sat[0][0] = GA_SAT_DEFAULT;
  gw_sat[0][0] = GW_SAT_DEFAULT;
  
  for (int i = 0; i<=2; i++){
    ba[i][0] = 0.0; 
    bw[i][0] = 0.0;
    sa[i][0] = 0.0; 
    sw[i][0] = 0.0;
  }
  
  for (int i = 0; i<=5; i++){
    ga[i][0] = 0.0; 
    gw[i][0] = 0.0;
  }
}

template <typename T, int nX>
void Imu_calibrator<T, nX>::gyro_bias_est(T w_in[3][1], bool set){
  if (!set){
    for (int i = 0; i<=2; i++){
      bw[i][0] = alpha*w_in[i][0] + beta*bw[i][0];
    } 
  }
  else{
	for (int i = 0; i<=2; i++){
      bw[i][0] = w_in[i][0];
    } 
  }
}

template <typename T, int nX>
void Imu_calibrator<T, nX>::set_alpha(T alpha_in){
  alpha = alpha_in;
  beta = (1.0-alpha);
}

template <typename T, int nX>
void Imu_calibrator<T, nX>::set_ba_sat(T ba_sat_in[3][1]){
  for (int i = 0; i<=2; i++){
    ba_sat[i][0] = ba_sat_in[i][0];
  }
}

template <typename T, int nX>
void Imu_calibrator<T, nX>::set_sa_sat(T sa_sat_in[3][1]){
  for (int i = 0; i<=2; i++){
    sa_sat[i][0] = sa_sat_in[i][0];  
  }
}

template <typename T, int nX>
void Imu_calibrator<T, nX>::set_bw_sat(T bw_sat_in[3][1]){
  for (int i = 0; i<=2; i++){
    bw_sat[i][0] = bw_sat_in[i][0];
  }
}

template <typename T, int nX>
void Imu_calibrator<T, nX>::set_sw_sat(T sw_sat_in[3][1]){
  for (int i = 0; i<=2; i++){
    sw_sat[i][0] = sw_sat_in[i][0];
  }
}

template <typename T, int nX>
void Imu_calibrator<T, nX>::set_ga_sat(T ga_sat_in){
  ga_sat[0][0] = ga_sat_in;
}

template <typename T, int nX>
void Imu_calibrator<T, nX>::set_gw_sat(T gw_sat_in){
  gw_sat[0][0] = gw_sat_in;
}


template <typename T, int nX>
void Imu_calibrator<T, nX>::update(T X[nX][1], bool upd_en){
  if (upd_en == true){
    if (nX > 9){
      for (int i = 9; i<=11; i++){
        ba[i-9][0] += X[i][0];
      }
      for (int i = 12; i<=14; i++){
        bw[i-12][0] += X[i][0];  
      }
    }
    if (nX > 15){
      for (int i = 15; i<=17; i++){
        sa[i-15][0] += X[i][0];  
      }
      for (int i = 18; i<=20; i++){
        sw[i-18][0] += X[i][0];  
      }
    }
    if (nX > 21){   
      for (int i = 21; i<=26; i++){
        ga[i-21][0] += X[i][0];  
      }
      for (int i = 27; i<=32; i++){
        gw[i-27][0] += X[i][0];  
      }
    }
    
    saturation(); 
  }
}

template <typename T, int nX>
void Imu_calibrator<T, nX>::saturation(void){
  for (int i = 0; i<=2; i++){
    //accel bias
    if (ba[i][0] > ba_sat[i][0]){ 
      ba[i][0] = ba_sat[i][0];
    }
    else if (ba[i][0] < -ba_sat[i][0]){
      ba[i][0] = -ba_sat[i][0];  
    }
    
    //accel scale
    if (sa[i][0] > sa_sat[i][0]){ 
      sa[i][0] = sa_sat[i][0];
    }
    else if (sa[i][0] < -sa_sat[i][0]){
      sa[i][0] = -sa_sat[i][0];  
    }
    
    //gyro bias
    if (bw[i][0] > bw_sat[i][0]){ 
      bw[i][0] = bw_sat[i][0];
    }
    else if (bw[i][0] < -bw_sat[i][0]){
      bw[i][0] = -bw_sat[i][0];  
    }
    
    //gyro scale
    if (sw[i][0] > sw_sat[i][0]){ 
      sw[i][0] = sw_sat[i][0];
    }
    else if (sw[i][0] < -sw_sat[i][0]){
      sw[i][0] = -sw_sat[i][0];  
    }
  }
    
  for (int i = 0; i<=5; i++){
    if (ga[i][0] > ga_sat[0][0]){ 
      ga[i][0] = ga_sat[0][0];
    }
    else if (ga[i][0] < -ga_sat[0][0]){
      ga[i][0] = -ga_sat[0][0];  
    }
    if (gw[i][0] > gw_sat[0][0]){ 
      gw[i][0] = gw_sat[0][0];
    }
    else if (gw[i][0] < -gw_sat[0][0]){
      gw[i][0] = -gw_sat[0][0];  
    }
  }  
}


template <typename T, int nX>
void Imu_calibrator<T, nX>::get_bias(T ba_out[3][1], T bw_out[3][1]){
  for (int i = 0; i<=2; i++){
    ba_out[i][0] = ba[i][0];
    bw_out[i][0] = bw[i][0];
  }
}

template <typename T, int nX>
void Imu_calibrator<T, nX>::get_scale(T am_out[3][3], T  wm_out[3][3]){
  T am_inv[3][3];
  T wm_inv[3][3];
  am_inv[0][0] = 1.0+sa[0][0]; am_inv[0][1] = ga[0][0];     am_inv[0][2] = ga[1][0];
  am_inv[1][0] = ga[2][0];     am_inv[1][1] = 1.0+sa[1][0]; am_inv[1][2] = ga[3][0];
  am_inv[2][0] = ga[4][0];     am_inv[2][1] = ga[5][0];     am_inv[2][2] = 1.0+sa[2][0];
  
  wm_inv[0][0] = 1.0+sw[0][0]; wm_inv[0][1] = gw[0][0];     wm_inv[0][2] = gw[1][0];
  wm_inv[1][0] = gw[2][0];     wm_inv[1][1] = 1.0+sw[1][0]; wm_inv[1][2] = gw[3][0];
  wm_inv[2][0] = gw[4][0];     wm_inv[2][1] = gw[5][0];     wm_inv[2][2] = 1.0+sw[2][0];
  
  m_inv<T,3>(am_out, am_inv);
  m_inv<T,3>(wm_out, wm_inv);
}

template <typename T, int nX>
void Imu_calibrator<T, nX>::calibration(T a_out[3][1], T w_out[3][1], T a_in[3][1], T w_in[3][1]){
  T ba[3][1], bw[3][1];
  T am[3][3], wm[3][3];
  T a_c[3][1], w_c[3][1]; 
  
  get_bias(ba, bw); 
  get_scale(am, wm);
  
  m_minus<T,3,1>(a_c,a_in,ba);
  m_minus<T,3,1>(w_c,w_in,bw);
  m_mul<T,3,3,1>(a_out,am,a_c);
  m_mul<T,3,3,1>(w_out,wm,w_c);
}


#endif //CALIBRATOR_HPP
