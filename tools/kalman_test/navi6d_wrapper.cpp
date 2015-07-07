#pragma GCC optimize "-O2"
#pragma GCC optimize "-ffast-math"
#pragma GCC optimize "-funroll-loops"
#pragma GCC diagnostic ignored "-Wdouble-promotion"

#define FAKE_SINS     FALSE

#include <math.h>

#include "../../firmware/lib/navi6d/navigator_sins.hpp"
#include "../../firmware/lib/navi6d/kalman_flags.cpp" // dirty hack allowing to not add this file to the Makefile

#include "../../firmware/lib/mavlink/C/lapwing/mavlink.h"
#include "../../firmware/lib/uav_utils/geometry.hpp"
#include "navi6d_wrapper.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define MAIN_SENSOR_ADIS    FALSE

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

#if ! FAKE_SINS
static NavigatorSins<klmnfp, 15, 17> nav_sins;
static InitParams<klmnfp> init_params;
static CalibParams<klmnfp> calib_params;
static KalmanParams<klmnfp> kalman_params;
static RefParams<klmnfp> ref_params;
#endif

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
/**
 *
 */
static void unpack_test_data(baro_data_t &baro,
                             odometer_data_t &odo,
                             marg_data_t &marg,
                             gnss::gnss_data_t &gps,
                             const mavlink_navi6d_debug_input_t &test) {

  baro.alt  = test.baro_alt;
  odo.speed = test.odo_speed;
  marg.dT   = test.marg_dt;
  for (size_t i=0; i<3; i++) {
    marg.acc[i] = test.marg_acc[i];
    marg.gyr[i] = test.marg_gyr[i];
    marg.mag[i] = test.marg_mag[i];
  }

  gps.latitude    = test.gnss_lat;
  gps.longitude   = test.gnss_lon;
  gps.altitude    = test.gnss_alt;
  gps.course      = test.gnss_course;
  gps.fix         = test.gnss_fix_type;
  gps.fresh       = test.gnss_fresh;
  gps.speed_type  = (gnss::speed_t)test.gnss_speed_type;
  gps.speed       = test.gnss_speed;
  for (size_t i=0; i<3; i++) {
    gps.v[i] = test.gnss_v[i];
  }
}

/**
 *
 */
static bool dbg_out_verify(const NaviData<klmnfp> &data,
                           const mavlink_navi6d_debug_output_t &ref) {

  if ((ref.roll != data.eu_nv[0][0]) ||
      (ref.pitch!= data.eu_nv[1][0]) ||
      (ref.yaw  != data.eu_nv[2][0]) ||
      (ref.lat  != data.r[0][0]) ||
      (ref.lon  != data.r[1][0]) ||
      (ref.alt  != data.r[2][0])) {
    return false;
  }
  else
    return true;
}

/**
 *
 */
void Navi6dWrapper::prepare_data_gnss(gnss::gnss_data_t &gnss_data) {

  if ((*en_gnss == 1) && (gnss_data.fresh) && (gnss_data.fix > 0)) {
    nav_sins.sensor_data.r_sns[0][0] = deg2rad(gnss_data.latitude);
    nav_sins.sensor_data.r_sns[1][0] = deg2rad(gnss_data.longitude);
    nav_sins.sensor_data.r_sns[2][0] = gnss_data.altitude;
    nav_sins.sensor_flags.sns_r_en = true;
    nav_sins.sensor_flags.sns_h_en = true;
    if (*en_gnss_v == 1) {
      switch(gnss_data.speed_type) {
      case gnss::speed_t::SPEED_COURSE:
        nav_sins.sensor_data.v_sns[0][0] = gnss_data.speed * cos(deg2rad(gnss_data.course));
        nav_sins.sensor_data.v_sns[1][0] = gnss_data.speed * sin(deg2rad(gnss_data.course));
        nav_sins.sensor_data.v_sns[2][0] = gnss_data.course;
        nav_sins.sensor_flags.sns_v_n_en = true;
        nav_sins.sensor_flags.sns_v_e_en = true;
        nav_sins.sensor_flags.sns_v_d_en = false;
        break;
      case gnss::speed_t::VECTOR_3D:
      case gnss::speed_t::BOTH:
        for (size_t i=0; i<3; i++) {
          nav_sins.sensor_data.v_sns[i][0] = gnss_data.v[i];
        }
        nav_sins.sensor_flags.sns_v_n_en = true;
        nav_sins.sensor_flags.sns_v_e_en = true;
        nav_sins.sensor_flags.sns_v_d_en = true;
        break;
      default:
        nav_sins.sensor_flags.sns_v_n_en = false;
        nav_sins.sensor_flags.sns_v_e_en = false;
        nav_sins.sensor_flags.sns_v_d_en = false;
        break;
      }
    }
  }

  // Important! Must be set to false after data processing
  if (gnss_data.fresh) {
    gnss_data.fresh = false;
  }
}

/*
 *
 */
void Navi6dWrapper::prepare_data(const baro_data_t &baro,
                                 const odometer_data_t &speed,
                                 const marg_data_t &marg,
                                 gnss::gnss_data_t &gps) {

  prepare_data_gnss(gps);

  if (*en_zihr == 1) {
    nav_sins.sensor_flags.zihr_en = true;
  }

  if (*en_odo == 1) {
    nav_sins.sensor_data.v_odo[0][0] = speed.speed;
    nav_sins.sensor_data.v_odo[1][0] = 0;
    nav_sins.sensor_data.v_odo[2][0] = 0;
    nav_sins.sensor_flags.odo_en = true;
  }

  if (*en_zupt == 1) {
    nav_sins.sensor_flags.zupt_en = true;
  }

  if (*en_nonhol == 1) {
    nav_sins.sensor_flags.nonhol_y_en = true;
    nav_sins.sensor_flags.nonhol_z_en = true;
  }

  if (*en_mag == 1) {
    nav_sins.sensor_flags.mag_en = true;
  }

  if (*en_euler == 1) {
    if (nav_sins.glrt_det.status){
      nav_sins.sensor_flags.course_en = true;
    }
  }
  nav_sins.sensor_flags.baro_fix_en = true;
  if (*en_baro == 1) {
    nav_sins.sensor_flags.alt_b_en = true;
    nav_sins.sensor_data.alt_b[0][0] = baro.alt;
  }

  for(size_t i=0; i<3; i++) {
    nav_sins.sensor_data.fb[i][0] = marg.acc[i];
    nav_sins.sensor_data.wb[i][0] = marg.gyr[i];
    nav_sins.sensor_data.mb[i][0] = marg.mag[i];
  }
}

/**
 *
 */
void Navi6dWrapper::read_settings(void) {

}

/**
 *
 */
void Navi6dWrapper::sins_cold_start(void) {

  ref_params.eu_vh_base[0][0] = deg2rad(*eu_vh_roll);
  ref_params.eu_vh_base[1][0] = deg2rad(*eu_vh_pitch);
  ref_params.eu_vh_base[2][0] = deg2rad(*eu_vh_yaw);

  init_params.est_gyro_bias = true;
  //init_params.sigma_Pi[0][0] = 200; //initial position STD (m)
  init_params.sigma_Pi[3][0] = M_PI; //initial heading STD (rad)
  init_params.dT = this->dT_cache;
  init_params.rst_dT = 0.5;

  kalman_params.sigma_R[0][0] = *R_ne_sns; //ne_sns
  kalman_params.sigma_R[1][0] = *R_d_sns; //d_sns
  kalman_params.sigma_R[2][0] = *R_v_n_sns; //v_n_sns
  kalman_params.sigma_R[3][0] = *R_odo; //odo
  kalman_params.sigma_R[4][0] = *R_nonhol; //nonhol
  kalman_params.sigma_R[5][0] = *R_baro; //baro
  kalman_params.sigma_R[6][0] = *R_mag; //mag
  kalman_params.sigma_R[7][0] = *R_euler; //roll,pitch,yaw (rad)
  kalman_params.sigma_R[8][0] = *R_zihr; // zihr

  kalman_params.sigma_Qm[0][0] = *Qm_acc; //acc
  kalman_params.sigma_Qm[1][0] = *Qm_gyr; //gyr
  kalman_params.sigma_Qm[2][0] = *Qm_acc_x; //acc_x
  kalman_params.sigma_Qm[3][0] = *Qm_acc_y; //acc_y
  kalman_params.sigma_Qm[4][0] = *Qm_acc_z; //acc_z
  kalman_params.sigma_Qm[5][0] = *Qm_gyr_bias; //gyr_bias

  calib_params.ba[0][0] = *acc_bias_x;
  calib_params.ba[1][0] = *acc_bias_y;
  calib_params.ba[2][0] = *acc_bias_z;

  calib_params.bw[0][0] = *gyr_bias_x;
  calib_params.bw[1][0] = *gyr_bias_y;
  calib_params.bw[2][0] = *gyr_bias_z;

  calib_params.sa[0][0] = *acc_scale_x;
  calib_params.sa[1][0] = *acc_scale_y;
  calib_params.sa[2][0] = *acc_scale_z;

  calib_params.sw[0][0] = *gyr_scale_x;
  calib_params.sw[1][0] = *gyr_scale_y;
  calib_params.sw[2][0] = *gyr_scale_z;

  calib_params.bm[0][0] = -3.79611/1000;
  calib_params.bm[1][0] = 15.2098/1000;
  calib_params.bm[2][0] = -5.45266/1000;

  calib_params.m_s[0][0] = 0.916692;
  calib_params.m_s[1][0] = 0.912;
  calib_params.m_s[2][0] = 0.9896;

  calib_params.m_no[0][0] = -0.0031;
  calib_params.m_no[1][0] = 0.0078;
  calib_params.m_no[2][0] = 0.0018;

  nav_sins.set_init_params(init_params);
  nav_sins.set_calib_params(calib_params);
  nav_sins.set_kalman_params(kalman_params);
  nav_sins.set_ref_params(ref_params);

  nav_sins.command_executor(1);
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
Navi6dWrapper::Navi6dWrapper(void) {
  return;
}

/**
 *
 */
void Navi6dWrapper::start(void) {

  read_settings();
  restart_cache = *restart + 1; // enforce sins restart in first update call

  ready = true;
}

/**
 *
 */
void Navi6dWrapper::stop(void) {
  ready = false;
}

/**
 *
 */
void Navi6dWrapper::update(const mavlink_navi6d_debug_input_t &test,
                           const mavlink_navi6d_debug_output_t &ref) {

  baro_data_t baro;
  odometer_data_t odo;
  marg_data_t marg;
  gnss::gnss_data_t gps;

  unpack_test_data(baro, odo, marg, gps, test);

  /* reapply new dT if needed */
  if (this->dT_cache != marg.dT) {
    this->dT_cache = marg.dT;
    init_params.dT = marg.dT;
    init_params.rst_dT = 0.5;
    nav_sins.set_init_params(init_params);
  }

  /* restart sins if requested */
  if (*restart != restart_cache) {
    sins_cold_start();
    restart_cache = *restart;
  }

  prepare_data(baro, odo, marg, gps);
  nav_sins.run();

  dbg_out_verify(nav_sins.navi_data, ref);
}



