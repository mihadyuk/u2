#ifndef NAVIGATOR_SINS_HPP
#define NAVIGATOR_SINS_HPP

#include "ahrs_simple.hpp"
#include "sins_kalman_loose.hpp"

#define LAT_DEFAULT 0.92f
#define LON_DEFAULT 0.48f
#define ALT_DEFAULT 255.0f

#define DEFAULT_INIT_TIME 5.0f
#define ACCEL_BIAS_SAT_DEFAULT 1.0f
#define GYRO_BIAS_SAT_DEFAULT 10.0f*DEG2RAD
#define BIAS_FILTER_ALPHA_DEFAULT 0.05f
#define MAG_DEC_DEFAULT 7.0f*DEG2RAD
#define MAG_INC_DEFAULT 69.0f*DEG2RAD
#define FREQ_DEFAULT 40.0f
#define DT_DEFAULT 1.0/FREQ_DEFAULT
#define RST_CNT_DEFAULT 50
#define AUTO_INIT_ALPHA_DEFAULT 0.5
#define SNS_EXTRAPOLATION false

template <typename T>
class SensorData{
public:
  T r_sns[3][1];
  T v_sns[3][1];
  T alt_b[1][1];
  T v_odo[3][1];
  T mb[3][1];
  T ang[3][1];
  T fb[3][1];
  T wb[3][1];
  bool pps;
};

template <typename T>
class CalibParams{
public:
  T ba_sat[3][1];
  T bw_sat[3][1];
  T sa_sat[3][1];
  T sw_sat[3][1];
  T ga_sat;
  T gw_sat;
  T alpha;
};

template <typename T>
class RefParams{
public:
  T mn[3][1];
  T mag_dec;
  T mag_inc;
  T mag_ang[3][1];
  T eu_vh_base[3][1];
  T eu_vh_turret[3][1];
  T eu_hb[3][1];
  T lever_arm_sns[3][1];
  T lever_arm_wheel[3][1];
  bool sns_extr_en;
  void update(void);
};

template <typename T>
void RefParams<T>::update(void){
  mn[0][0] = 1.0;
  mn[1][0] = 0.0;
  mn[2][0] = 0.0;
  mag_ang[0][0] = 0.0;
  mag_ang[1][0] = -mag_inc;
  mag_ang[2][0] = mag_dec;
  lever_arm_sns[0][0] = 0.0;
  lever_arm_sns[1][0] = 0.0;
  lever_arm_sns[2][0] = 0.0;
  lever_arm_wheel[0][0] = 0.0;
  lever_arm_wheel[1][0] = 0.0;
  lever_arm_wheel[2][0] = 0.0;
}




template <typename T>
class InitParams{
public:
  T r_init[3][1];
  T v_init[3][1];
  T qnv_init[4][1];
  T eu_nv_init[3][1];
  
  T alpha;
  T Pi[10][1];
  T dT;
  T init_time;
  unsigned long rst_cnt;
};

template <typename T>
class KalmanParams{
public:
  T R[9][1];
  T Qm[10][1];
};


template <typename T>
class NaviData{
public:
  T r[3][1];
  T v[3][1];
  T qnv[4][1];
  T qnb[4][1];
  T eu_nv[3][1];
  T eu_nb[3][1];
  T course_v[1][1];
  T mag_head_v[1][1];
  T free_acc[3][1];
  T a_b[3][1];
  T w_b[3][1];
  unsigned long status;
};

template <typename T>
class AllParams{
public:
  InitParams<T> init_params;
  CalibParams<T> calib_params;
  KalmanParams<T> kalman_params;
  RefParams<T> ref_params;
};

typedef enum {
  NAV_IDLE = 0,
  NAV_INIT_PREASSIGNED_QUAT = 1,
  NAV_INIT_PREASSIGNED_EULER = 2,
  NAV_INIT_PREASSIGNED_COURSE = 3,
  NAV_INIT_AUTO = 4,
  NAV_WAIT_SRNS_DATA = 5,
  NAV_SINS_INIT = 6,
  NAV_RUN = 7
}navigator_mode_t;

template <typename T, int nX, int nY>
class NavigatorSins{
public:
  NavigatorSins(void);
  unsigned long status;
  AllParams<T> params;
  navigator_mode_t state;
  
  T r_init[3][1];
  T v_init[3][1];
  T qnv_init[4][1];
  
  T r_sns_extr[3][1];
  T v_sns_extr[3][1];
  //initial values
  T timer;
  
  Sins_Kalman_LC<T,nX,nY> Sins_lc;
  Ahrs_simple<T> Ahrs_smpl;
  KalmanFlags flg;
  
  NaviData<T> navi_data;
  
  void set_init_params(InitParams<T> init_params_in);
  void set_calib_params(CalibParams<T> calib_params_in);
  void set_kalman_params(KalmanParams<T> kalman_params_in);
  void set_ref_params(RefParams<T> ref_params_in);
  void set_all_params(AllParams<T> all_params_in);

  void flag_controller(KalmanFlags sensor_flags);                           
  void set_status(void);     
  void set_dT(void);
  void set_RQm(void);
  
  void command_executor(uint8_t cmd);
  
  void sns_extrapolator(T r_sns[3][1], T v_sns[3][1], bool pps);
  void get_navi_params(void);
 
  void initialize(navigator_mode_t mode);
  void run(SensorData<T> data, KalmanFlags sensor_flags);

  void update(T r_sns[3][1], T v_sns[3][1], T alt_b[1][1], T v_odo[3][1], T mb[3][1], T ang[3][1], T fb[3][1], T wb[3][1], T mn[3][1], bool pps);
};

template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::get_navi_params(void){
  for (int i = 0; i<3; i++){
    navi_data.r[i][0] = this->Sins_lc.r_sm[i][0];
    navi_data.v[i][0] = this->Sins_lc.v_sm[i][0];
    navi_data.eu_nv[i][0] = this->Sins_lc.eu_nv_out[i][0];
    navi_data.eu_nb[i][0] = this->Sins_lc.eu_sm[i][0];
  }
  for (int i = 0; i<4; i++){
   navi_data.qnv[i][0] = this->Sins_lc.qnv_out[i][0];
   navi_data.qnb[i][0] = this->Sins_lc.qnb_sm[i][0];
  }
  if (navi_data.v[0][0] != 0.0){
    navi_data.course_v[0][0] = atan2(navi_data.v[1][0],navi_data.v[0][0]); //course
  }
  else{
	navi_data.course_v[0][0] = 0.0; //course  
  }
  navi_data.mag_head_v[0][0] = this->Ahrs_smpl.eu[2][0];
  
  for (int i = 0; i<3; i++){
    navi_data.free_acc[i][0] = Sins_lc.acc_f[i][0];
    navi_data.a_b[i][0] = Sins_lc.fb_c[i][0];
    navi_data.w_b[i][0] = Sins_lc.wb_c[i][0];
  }
  navi_data.status = status;
}


template <typename T, int nX, int nY>
NavigatorSins<T, nX, nY>::NavigatorSins(void){
  state = NAV_IDLE;
  params.init_params.r_init[0][0] = LAT_DEFAULT;
  params.init_params.r_init[1][0] = LON_DEFAULT;
  params.init_params.r_init[2][0] = ALT_DEFAULT;
  params.init_params.qnv_init[0][0] = 1.0f;
  params.init_params.init_time = DEFAULT_INIT_TIME;
  
  qnv_init[0][0] = 1.0;
  for(int i = 0; i<3; i++)
  {
    params.init_params.v_init[i][0] = 0.0f;
	  params.init_params.eu_nv_init[i][0] = 0.0f;
    params.init_params.qnv_init[i+1][0] = 0.0f;
	  qnv_init[i+1][0] = 0.0f;
  }
  
  params.ref_params.mag_dec = MAG_DEC_DEFAULT; 
  params.ref_params.sns_extr_en = SNS_EXTRAPOLATION;
  params.init_params.dT = DT_DEFAULT;
  params.init_params.rst_cnt = RST_CNT_DEFAULT;
  params.init_params.alpha = AUTO_INIT_ALPHA_DEFAULT;
  status = 0;
}


////SET PARAMS
template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::set_init_params(InitParams<T> init_params_in){
  params.init_params = init_params_in;
}

template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::set_calib_params(CalibParams<T> calib_params_in){
  params.calib_params = calib_params_in;
  Sins_lc.Calib.set_ba_sat(params.calib_params.ba_sat);
  Sins_lc.Calib.set_bw_sat(params.calib_params.bw_sat);  
  Sins_lc.Calib.set_sa_sat(params.calib_params.sa_sat);
  Sins_lc.Calib.set_sw_sat(params.calib_params.sw_sat);
  Sins_lc.Calib.set_ga_sat(params.calib_params.ga_sat);
  Sins_lc.Calib.set_gw_sat(params.calib_params.gw_sat);
  Sins_lc.Calib.set_alpha(params.calib_params.alpha);                    
}

template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::set_kalman_params(KalmanParams<T> kalman_params_in){
  params.kalman_params = kalman_params_in;
  this->set_RQm();
}

template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::set_ref_params(RefParams<T> ref_params_in){
  params.ref_params = ref_params_in;
  params.ref_params.update();
  Ahrs_smpl.mag_dec = params.ref_params.mag_dec;
  
  Sins_lc.set_qvh_base(params.ref_params.eu_vh_base);
  Sins_lc.set_qvh_turret(params.ref_params.eu_vh_turret);
  Sins_lc.set_qhb(params.ref_params.eu_hb);
  Sins_lc.set_lever_arm(params.ref_params.lever_arm_sns, params.ref_params.lever_arm_wheel);
}
//////////  

template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::flag_controller(KalmanFlags sensor_flags){
  this->flg.sns_r_en = sensor_flags.sns_r_en;
  this->flg.sns_h_en = sensor_flags.sns_h_en;
  this->flg.sns_v_n_en = sensor_flags.sns_v_n_en;
  this->flg.sns_v_e_en = sensor_flags.sns_v_e_en;
  this->flg.sns_v_d_en = sensor_flags.sns_v_d_en;
  this->flg.odo_en = sensor_flags.odo_en;
  this->flg.nonhol_y_en = sensor_flags.nonhol_y_en;
  this->flg.nonhol_z_en = sensor_flags.nonhol_z_en;
  this->flg.alt_b_en = sensor_flags.alt_b_en;
  this->flg.baro_fix_en = sensor_flags.baro_fix_en;
  this->flg.mag_en = sensor_flags.mag_en;
  this->flg.roll_en = sensor_flags.roll_en;
  this->flg.pitch_en = sensor_flags.pitch_en;
  this->flg.course_en = sensor_flags.course_en;
  this->flg.zihr_en = sensor_flags.zihr_en;  
  
  // if ((sensor_flags.sns_r_en == true) || (sensor_flags.sns_v_en == true)){ //example
  // }
} 
 
template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::set_dT(void){
  Sins_lc.init_time_const(params.init_params.dT, params.init_params.rst_cnt);  
}

template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::set_RQm(void){
  Sins_lc.set_RQ(params.kalman_params.R,params.kalman_params.Qm);
}

template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::sns_extrapolator(T r_sns[3][1], T v_sns[3][1], bool pps){
  if (pps == true){
    Sins_lc.S.reset_extr();
  }
  
  if (params.ref_params.sns_extr_en == true){
    if (
        (this->flg.sns_r_en == true) || 
        (this->flg.sns_h_en == true) || 
        (this->flg.sns_v_n_en == true) || 
        (this->flg.sns_v_e_en == true) || 
        (this->flg.sns_v_d_en == true)
        ){
      m_plus<T,3,1>(r_sns_extr, r_sns, Sins_lc.S.dr_extr);
      m_plus<T,3,1>(v_sns_extr, v_sns, Sins_lc.S.dv_extr);
    }
    else{
      m_set<T,3,1>(r_sns_extr, 0.0);
      m_set<T,3,1>(v_sns_extr, 0.0);
    }
  }
  else{
    m_copy<T,3,1>(r_sns_extr, r_sns);
    m_copy<T,3,1>(v_sns_extr, v_sns);  
  }
}

template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::update(T r_sns[3][1], T v_sns[3][1], T alt_b[1][1], T v_odo[3][1], T mb[3][1], T ang[3][1], T fb[3][1], T wb[3][1], T mn[3][1], bool pps){
  timer += params.init_params.dT;  
  T alpha = params.init_params.alpha;
  T beta = 1.0-alpha;
  Ahrs_smpl.simple_update(fb, mb);
 
  this->sns_extrapolator(r_sns, v_sns, pps);
  
  switch (state){
    case NAV_IDLE:
    break;
    
    case NAV_INIT_AUTO:
	  m_copy<T,4,1>(qnv_init, Ahrs_smpl.qnv);
      Sins_lc.Calib.gyro_bias_est(wb, 1);
      state = NAV_WAIT_SRNS_DATA;
    break;
		
	  case NAV_WAIT_SRNS_DATA:
	    m_plus_alpha_beta<T,4,1>(qnv_init, Ahrs_smpl.qnv, qnv_init, alpha, beta);
      qnorm<T>(qnv_init);
	    Sins_lc.Calib.gyro_bias_est(wb, 0);
	    if ((flg.sns_r_en == true) && (this->timer > params.init_params.init_time)){
	      m_copy<T,3,1>(r_init, r_sns);
		    m_copy<T,3,1>(v_init, v_sns);	
        state = NAV_SINS_INIT;
      }   
	  break;

    case NAV_INIT_PREASSIGNED_QUAT:
	  case NAV_INIT_PREASSIGNED_EULER:
	    T quat_norm;	
	    m_copy<T,3,1>(r_init, params.init_params.r_init);
	    m_copy<T,3,1>(v_init, params.init_params.v_init); 
	    quat_norm = m_vec_norm<T,4>(params.init_params.qnv_init);
	    if ((state == NAV_INIT_PREASSIGNED_QUAT) && (quat_norm > 0.5)){
	      m_copy<T,4,1>(qnv_init, params.init_params.qnv_init);
	    }
	    else{
		    euler2quat<T>(qnv_init, params.init_params.eu_nv_init);
	    }
      state = NAV_SINS_INIT;
	  break;
	
	  case NAV_SINS_INIT:
	    Sins_lc.initialize(r_init, v_init, qnv_init,
                         params.kalman_params.R, params.kalman_params.Qm,
                         params.init_params.Pi, params.init_params.dT, params.init_params.rst_cnt);				 
      state = NAV_RUN;
	  break;
	
    case NAV_RUN:
	    this->set_RQm();
      this->set_dT();
      this->Sins_lc.update(r_sns_extr, v_sns_extr, alt_b, v_odo, mb, ang, fb, wb, mn, flg);
    break;
    
    default:
    break;
  }
  //this->Ahrs_smpl.mag_heading_update(Sins_lc.qnv_out, mb, Sins_lc.qbv);
  this->set_status();
  this->get_navi_params();
}


template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::set_status(void){
  this->status = 0x00000000;
  switch (state){
    case NAV_IDLE: 
      this->status |= 0;
    break;
    case NAV_INIT_AUTO: 
      this->status |= 1;
    break;
    case NAV_WAIT_SRNS_DATA: 
      this->status |= 2;
    break;
    case NAV_INIT_PREASSIGNED_QUAT: 
    case NAV_INIT_PREASSIGNED_EULER:
      this->status |= 3;
    break;
    case NAV_RUN: 
     this->status |= 4;
    break;
    default:
    break;
  }
  if (flg.sns_r_en == true){
    this->status |= 1<<4;  
  }
}

//////////////////////////////
template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::initialize(navigator_mode_t mode){
   timer = 0.0;
   this->state = mode;
   this->set_RQm();
   this->set_dT();
   this->Ahrs_smpl.mag_dec = params.ref_params.mag_dec;
   Sins_lc.set_P(params.init_params.Pi);
}

    
template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::run(SensorData<T> data, KalmanFlags sensor_flags)
{                                         
  this->flag_controller(sensor_flags);
  this->update(data.r_sns, data.v_sns, data.alt_b, data.v_odo, data.mb, data.ang, data.fb, data.wb, params.ref_params.mn, data.pps);
};

template <typename T, int nX, int nY>
void NavigatorSins<T, nX, nY>::command_executor(uint8_t cmd)
{
  switch (cmd)
  {
    case 0:  
    break;
    case 1:
      this->initialize(NAV_INIT_AUTO); ; 
    break;
    case 2:
      this->initialize(NAV_INIT_PREASSIGNED_EULER);  
    break;
    case 3:
      this->initialize(NAV_INIT_PREASSIGNED_QUAT);  
    break;
    case 4:
      this->initialize(NAV_INIT_PREASSIGNED_COURSE);  
    break;
    default:
    break;
  }
};

#endif //NAVIGATOR_HPP
