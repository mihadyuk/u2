#pragma GCC optimize "-O2"
#pragma GCC diagnostic ignored "-Wdouble-promotion"

#define FAKE_SINS     FALSE

#include <math.h>
#include "main.h"
#include "math_f.hpp"

#if ! FAKE_SINS
#include "navigator_sins.hpp"
#include "kalman_flags.cpp" // dirty hack allowing to not add this file to the Makefile
#endif

#include "navi6d_wrapper.hpp"
#include "mavlink_local.hpp"
#include "acs_input.hpp"
#include "ublox.hpp"
#include "geometry.hpp"
#include "time_keeper.hpp"
#include "param_registry.hpp"
#include "pads.h"

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
extern mavlink_debug_t                 mavlink_out_debug_struct;
extern mavlink_debug_vect_t            mavlink_out_debug_vect_struct;
extern mavlink_highres_imu_t           mavlink_out_highres_imu_struct;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
typedef float klmnfp;
#if ! FAKE_SINS
__CCM__ static NavigatorSins<klmnfp, 15, 16> nav_sins;
__CCM__ static InitParams<klmnfp> init_params;
__CCM__ static CalibParams<klmnfp> calib_params;
__CCM__ static KalmanParams<klmnfp> kalman_params;
__CCM__ static RefParams<klmnfp> ref_params;
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
void Navi6dWrapper::prepare_gnss(const odometer_data_t &speed) {
#if ! FAKE_SINS
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

  nav_sins.sensor_data.v_odo[0][0] = speed.speed;
  nav_sins.sensor_data.v_odo[1][0] = 0;
  nav_sins.sensor_data.v_odo[2][0] = 0;

  // Important! Must be set to false after data processing
  if (gnss_data.fresh) {
    gnss_data.fresh = false;
  }
#else
  (void)speed;
#endif
}

/*
 *
 */
#if ! FAKE_SINS
void Navi6dWrapper::prepare_data(const baro_data_t &abs_press,
                                 const odometer_data_t &speed,
                                 const marg_data_t &marg)
{
  prepare_gnss(speed);

  if ((0 == speed.speed) && (1 == *en_zihr)) {
    nav_sins.sensor_flags.zihr_en = true;
  }

  if (nav_sins.glrt_det.status){
    nav_sins.sensor_data.ang[2][0] = nav_sins.navi_data.eu_nv[2][0];
  }

  if (*en_odo == 1) {
    nav_sins.sensor_flags.odo_en = true;
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
    nav_sins.sensor_data.alt_b[0][0] = abs_press.alt;
  }

  for(size_t i=0; i<3; i++) {
    nav_sins.sensor_data.fb[i][0] = marg.acc[i];
    nav_sins.sensor_data.wb[i][0] = marg.gyr[i];
    nav_sins.sensor_data.mb[i][0] = marg.mag[i];
  }
}
#else
void Navi6dWrapper::prepare_data(const baro_data_t &abs_press,
                                 const odometer_data_t &speed,
                                 const marg_data_t &marg) {
  (void)abs_press;
  (void)speed;
  (void)marg;
}
#endif

/**
 *
 */
#if ! FAKE_SINS
void Navi6dWrapper::navi2mavlink(void) {

  const NaviData<klmnfp> &data = nav_sins.navi_data;

  mavlink_out_highres_imu_struct.xacc = data.fb_c[0][0];
  mavlink_out_highres_imu_struct.yacc = data.fb_c[1][0];
  mavlink_out_highres_imu_struct.zacc = data.fb_c[2][0];

  mavlink_out_highres_imu_struct.xgyro = data.wb_c[0][0];
  mavlink_out_highres_imu_struct.ygyro = data.wb_c[1][0];
  mavlink_out_highres_imu_struct.zgyro = data.wb_c[2][0];

  mavlink_out_highres_imu_struct.xmag = data.mb_c[0][0];
  mavlink_out_highres_imu_struct.ymag = data.mb_c[1][0];
  mavlink_out_highres_imu_struct.zmag = data.mb_c[2][0];

  mavlink_out_highres_imu_struct.time_usec = TimeKeeper::utc();
}
#endif

/**
 *
 */
#if ! FAKE_SINS
void Navi6dWrapper::navi2acs(void) {

  const NaviData<klmnfp> &data = nav_sins.navi_data;

  acs_in.ch[ACS_INPUT_roll] = data.eu_nv[0][0];
  acs_in.ch[ACS_INPUT_pitch]= data.eu_nv[1][0];
  acs_in.ch[ACS_INPUT_yaw]  = data.eu_nv[2][0];

  acs_in.ch[ACS_INPUT_lat] = data.r[0][0];
  acs_in.ch[ACS_INPUT_lon] = data.r[1][0];
  acs_in.ch[ACS_INPUT_alt] = data.r[2][0];

  acs_in.ch[ACS_INPUT_vx] = data.v[0][0];
  acs_in.ch[ACS_INPUT_vy] = data.v[1][0];
  acs_in.ch[ACS_INPUT_vz] = data.v[2][0];

  acs_in.ch[ACS_INPUT_q0] = data.qnv[0][0];
  acs_in.ch[ACS_INPUT_q1] = data.qnv[1][0];
  acs_in.ch[ACS_INPUT_q2] = data.qnv[2][0];
  acs_in.ch[ACS_INPUT_q3] = data.qnv[3][0];

  acs_in.ch[ACS_INPUT_ax_body] = data.a_b[0][0];
  acs_in.ch[ACS_INPUT_ay_body] = data.a_b[1][0];
  acs_in.ch[ACS_INPUT_az_body] = data.a_b[2][0];

  acs_in.ch[ACS_INPUT_wx] = data.w_b[0][0];
  acs_in.ch[ACS_INPUT_wy] = data.w_b[1][0];
  acs_in.ch[ACS_INPUT_wz] = data.w_b[2][0];

  acs_in.ch[ACS_INPUT_free_ax] = data.free_acc[0][0];
  acs_in.ch[ACS_INPUT_free_ay] = data.free_acc[1][0];
  acs_in.ch[ACS_INPUT_free_az] = data.free_acc[2][0];

  mavlink_out_debug_struct.value = data.mag_mod;
}
#endif

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
Navi6dWrapper::Navi6dWrapper(ACSInput &acs_in, gnss::GNSSReceiver &GNSS) :
acs_in(acs_in),
GNSS(GNSS)
{
  return;
}

/**
 *
 */
void Navi6dWrapper::start(float dT) {
#if ! FAKE_SINS
  gnss_data.fresh = false;
  GNSS.subscribe(&gnss_data);

  param_registry.valueSearch("SINS_en_gnss",   &en_gnss);
  param_registry.valueSearch("SINS_en_odo",    &en_odo);
  param_registry.valueSearch("SINS_en_baro",   &en_baro);
  param_registry.valueSearch("SINS_en_euler",  &en_euler);
  param_registry.valueSearch("SINS_en_mag",    &en_mag);
  param_registry.valueSearch("SINS_en_nonhol", &en_nonhol);
  param_registry.valueSearch("SINS_en_zihr",   &en_zihr);
  param_registry.valueSearch("SINS_en_gnss_v", &en_gnss_v);

  param_registry.valueSearch("SINS_R_ne_sns",   &R_ne_sns);
  param_registry.valueSearch("SINS_R_d_sns",    &R_d_sns);
  param_registry.valueSearch("SINS_R_v_n_sns",  &R_v_n_sns);
  param_registry.valueSearch("SINS_R_odo",      &R_odo);
  param_registry.valueSearch("SINS_R_nonhol",   &R_nonhol);
  param_registry.valueSearch("SINS_R_baro",     &R_baro);
  param_registry.valueSearch("SINS_R_mag",      &R_mag);
  param_registry.valueSearch("SINS_R_euler",    &R_euler);
  param_registry.valueSearch("SINS_R_zihr",     &R_zihr);

  param_registry.valueSearch("SINS_Qm_acc",     &Qm_acc);
  param_registry.valueSearch("SINS_Qm_gyr",     &Qm_gyr);
  param_registry.valueSearch("SINS_Qm_acc_x",   &Qm_acc_x);
  param_registry.valueSearch("SINS_Qm_acc_y",   &Qm_acc_y);
  param_registry.valueSearch("SINS_Qm_acc_z",   &Qm_acc_z);
  param_registry.valueSearch("SINS_Qm_gyr_bias",&Qm_gyr_bias);

  param_registry.valueSearch("SINS_eu_vh_roll", &eu_vh_roll);
  param_registry.valueSearch("SINS_eu_vh_pitch",&eu_vh_pitch);
  param_registry.valueSearch("SINS_eu_vh_yaw",  &eu_vh_yaw);

  param_registry.valueSearch("GLRT_acc_sigma",  &acc_sigma);
  param_registry.valueSearch("GLRT_gyr_sigma",  &gyr_sigma);
  param_registry.valueSearch("GLRT_gamma",      &gamma);
  param_registry.valueSearch("GLRT_samples",    &samples);

  ref_params.eu_vh_base[0][0] = deg2rad(*eu_vh_roll);
  ref_params.eu_vh_base[1][0] = deg2rad(*eu_vh_pitch);
  ref_params.eu_vh_base[2][0] = deg2rad(*eu_vh_yaw);

#if MAIN_SENSOR_ADIS
  init_params.est_gyro_bias = false;
#else
  init_params.est_gyro_bias = true;
#endif

  //init_params.sigma_Pi[0][0] = 200; //initial position STD (m)
  init_params.sigma_Pi[3][0] = M_PI; //initial heading STD (rad)

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

  init_params.dT = dT;

  nav_sins.set_init_params(init_params);
  nav_sins.set_calib_params(calib_params);
  nav_sins.set_kalman_params(kalman_params);
  nav_sins.set_ref_params(ref_params);

  nav_sins.command_executor(1);

  ready = true;

#else
  (void)dT;
#endif
}

/**
 *
 */
void Navi6dWrapper::stop(void) {
  ready = false;
  GNSS.unsubscribe(&gnss_data);
}

/**
 *
 */
void Navi6dWrapper::update(const baro_data_t &abs_press,
                           const odometer_data_t &speed,
                           const marg_data_t &marg)
{
#if ! FAKE_SINS
  osalDbgCheck(ready);

  nav_sins.set_ref_params(ref_params);

  prepare_data(abs_press, speed, marg);
  nav_sins.run();
  navi2acs();
  navi2mavlink();

  mavlink_out_debug_vect_struct.time_usec = TimeKeeper::utc();
  mavlink_out_debug_vect_struct.x = nav_sins.navi_data.a_bias[0][0];
  mavlink_out_debug_vect_struct.y = nav_sins.navi_data.a_bias[1][0];
  mavlink_out_debug_vect_struct.z = nav_sins.navi_data.a_bias[2][0];

#else
  (void)abs_press;
  (void)speed;
  (void)marg;
#endif
}



