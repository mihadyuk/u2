#pragma GCC optimize "-O2"
#pragma GCC optimize "-ffast-math"
#pragma GCC optimize "-funroll-loops"
#pragma GCC diagnostic ignored "-Wdouble-promotion"

#define FAKE_SINS     FALSE

#include <math.h>
#include "main.h"

#if ! FAKE_SINS
#include "navigator_sins.hpp"
#include "kalman_flags.cpp" // dirty hack allowing to not add this file to the Makefile
#endif

#include "navi6d_wrapper.hpp"
#include "mavlink_local.hpp"
#include "acs_input.hpp"
#include "geometry.hpp"
#include "time_keeper.hpp"
#include "param_registry.hpp"
#include "mav_logger.hpp"

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

extern MavLogger mav_logger;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

#if ! FAKE_SINS
__CCM__ static NavigatorSins<klmnfp, 15, 17> nav_sins;
__CCM__ static InitParams<klmnfp> init_params;
__CCM__ static CalibParams<klmnfp> calib_params;
__CCM__ static KalmanParams<klmnfp> kalman_params;
__CCM__ static RefParams<klmnfp> ref_params;
#endif

__CCM__ static mavlink_navi6d_debug_input_t   dbg_in_struct;
__CCM__ static mavlink_navi6d_debug_output_t  dbg_out_struct;
__CCM__ static mavMail dbg_in_mail;
__CCM__ static mavMail dbg_out_mail;

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
static void dbg_in_fill_gnss(const gnss::gnss_data_t &data) {

  dbg_in_struct.gnss_lat        = data.latitude;
  dbg_in_struct.gnss_lon        = data.longitude;
  dbg_in_struct.gnss_alt        = data.altitude;
  dbg_in_struct.gnss_course     = data.course;
  dbg_in_struct.gnss_fix_type   = data.fix;
  dbg_in_struct.gnss_fresh      = data.fresh;
  dbg_in_struct.gnss_speed_type = (uint8_t)data.speed_type;
  dbg_in_struct.gnss_speed      = data.speed;
  for (size_t i=0; i<3; i++) {
    dbg_in_struct.gnss_v[i]     = data.v[i];
  }

  dbg_in_struct.time_boot_ms    = TIME_BOOT_MS;
}

/**
 *
 */
static void dbg_in_fill_other(const baro_data_t &baro,
                              const odometer_data_t &odo,
                              const marg_data_t &marg) {

  dbg_in_struct.baro_alt  = baro.alt;
  dbg_in_struct.odo_speed = odo.speed;
  dbg_in_struct.marg_dt   = marg.dT;
  for (size_t i=0; i<3; i++) {
    dbg_in_struct.marg_acc[i] = marg.acc[i];
    dbg_in_struct.marg_gyr[i] = marg.gyr[i];
    dbg_in_struct.marg_mag[i] = marg.mag[i];
  }

  dbg_in_struct.time_boot_ms    = TIME_BOOT_MS;
}

/**
 *
 */
static void dbg_in_append_log(void) {
  if (dbg_in_mail.free()) {
    dbg_in_mail.fill(&dbg_in_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT);
    mav_logger.write(&dbg_in_mail);
  }
}

/**
 *
 */
static void dbg_out_fill(const NaviData<klmnfp> &data) {

  dbg_out_struct.roll = data.eu_nv[0][0];
  dbg_out_struct.pitch= data.eu_nv[1][0];
  dbg_out_struct.yaw  = data.eu_nv[2][0];

  dbg_out_struct.lat  = data.r[0][0];
  dbg_out_struct.lon  = data.r[1][0];
  dbg_out_struct.alt  = data.r[2][0];

  dbg_out_struct.time_boot_ms = TIME_BOOT_MS;
}

/**
 *
 */
static void dbg_out_append_log(void) {
  if (dbg_out_mail.free()) {
    dbg_out_mail.fill(&dbg_out_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT);
    mav_logger.write(&dbg_out_mail);
  }
}

/**
 *
 */
void Navi6dWrapper::prepare_data_gnss(gnss::gnss_data_t &gnss_data) {
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

  // Important! Must be set to false after data processing
  if (gnss_data.fresh) {
    gnss_data.fresh = false;
  }
#endif
}

/*
 *
 */
#if ! FAKE_SINS
void Navi6dWrapper::prepare_data(const baro_data_t &baro,
                                 const odometer_data_t &odo,
                                 const marg_data_t &marg) {

  dbg_in_fill_gnss(this->gps);
  prepare_data_gnss(this->gps);

  dbg_in_fill_other(baro, odo, marg);
  dbg_in_append_log();

  if (*en_zihr == 1) {
    nav_sins.sensor_flags.zihr_en = true;
  }

  if (*en_odo == 1) {
    nav_sins.sensor_data.v_odo[0][0] = odo.speed;
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
void Navi6dWrapper::debug2mavlink(void) {
  mavlink_out_debug_vect_struct.time_usec = TimeKeeper::utc();
  /*  mavlink_out_debug_vect_struct.x = nav_sins.navi_data.a_bias[0][0];
  mavlink_out_debug_vect_struct.y = nav_sins.navi_data.a_bias[1][0];
  mavlink_out_debug_vect_struct.z = nav_sins.navi_data.a_bias[2][0];
  */
  /*   mavlink_out_debug_vect_struct.x = nav_sins.sensor_data.v_sns[0][0];
    mavlink_out_debug_vect_struct.y = nav_sins.sensor_data.v_sns[1][0];
    mavlink_out_debug_vect_struct.z = nav_sins.sensor_data.v_sns[2][0];*/
    /*mavlink_out_debug_vect_struct.x = sqrt(nav_sins.navi_data.mb_c[0][0]*nav_sins.navi_data.mb_c[0][0]+
        nav_sins.navi_data.mb_c[1][0]*nav_sins.navi_data.mb_c[1][0]+
        nav_sins.navi_data.mb_c[2][0]*nav_sins.navi_data.mb_c[2][0]);*/
    mavlink_out_debug_vect_struct.x = nav_sins.navi_data.mb_c[0][0];
    mavlink_out_debug_vect_struct.y = nav_sins.navi_data.mb_c[1][0];
    mavlink_out_debug_vect_struct.z = nav_sins.navi_data.mb_c[2][0];
  //mavlink_out_debug_vect_struct.x = nav_sins.glrt_det.test_stat;
  mavlink_out_debug_struct.value = nav_sins.navi_data.mag_mod;
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
}
#endif

/**
 *
 */
void Navi6dWrapper::read_settings(void) {

  param_registry.valueSearch("SINS_en_gnss",    &en_gnss);
  param_registry.valueSearch("SINS_en_odo",     &en_odo);
  param_registry.valueSearch("SINS_en_baro",    &en_baro);
  param_registry.valueSearch("SINS_en_euler",   &en_euler);
  param_registry.valueSearch("SINS_en_mag",     &en_mag);
  param_registry.valueSearch("SINS_en_nonhol",  &en_nonhol);
  param_registry.valueSearch("SINS_en_zihr",    &en_zihr);
  param_registry.valueSearch("SINS_en_gnss_v",  &en_gnss_v);
  param_registry.valueSearch("SINS_en_zupt",    &en_zupt);

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

  param_registry.valueSearch("SINS_acc_bias_x", &acc_bias_x);
  param_registry.valueSearch("SINS_acc_bias_y", &acc_bias_y);
  param_registry.valueSearch("SINS_acc_bias_z", &acc_bias_z);

  param_registry.valueSearch("SINS_gyr_bias_x", &gyr_bias_x);
  param_registry.valueSearch("SINS_gyr_bias_y", &gyr_bias_y);
  param_registry.valueSearch("SINS_gyr_bias_z", &gyr_bias_z);

  param_registry.valueSearch("SINS_acc_scale_x",&acc_scale_x);
  param_registry.valueSearch("SINS_acc_scale_y",&acc_scale_y);
  param_registry.valueSearch("SINS_acc_scale_z",&acc_scale_z);

  param_registry.valueSearch("SINS_gyr_scale_x",&gyr_scale_x);
  param_registry.valueSearch("SINS_gyr_scale_y",&gyr_scale_y);
  param_registry.valueSearch("SINS_gyr_scale_z",&gyr_scale_z);

  param_registry.valueSearch("SINS_eu_vh_roll", &eu_vh_roll);
  param_registry.valueSearch("SINS_eu_vh_pitch",&eu_vh_pitch);
  param_registry.valueSearch("SINS_eu_vh_yaw",  &eu_vh_yaw);

  param_registry.valueSearch("GLRT_acc_sigma",  &acc_sigma);
  param_registry.valueSearch("GLRT_gyr_sigma",  &gyr_sigma);
  param_registry.valueSearch("GLRT_gamma",      &gamma);
  param_registry.valueSearch("GLRT_samples",    &samples);

  param_registry.valueSearch("SINS_restart",    &restart);
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
Navi6dWrapper::Navi6dWrapper(ACSInput &acs_in, gnss::GNSSReceiver &GNSS) :
acs_in(acs_in),
GNSS(GNSS)
{
  return;
}

/**
 *
 */
void Navi6dWrapper::start(void) {
#if ! FAKE_SINS
  gps.fresh = false;
  GNSS.subscribe(&gps);

  read_settings();
  restart_cache = *restart + 1; // enforce sins restart in first update call

  ready = true;
#endif
}

/**
 *
 */
void Navi6dWrapper::stop(void) {
  ready = false;
  GNSS.unsubscribe(&gps);
}

/**
 *
 */
void Navi6dWrapper::update(const baro_data_t &baro,
                           const odometer_data_t &odo,
                           const marg_data_t &marg)
{
#if ! FAKE_SINS
  osalDbgCheck(ready);

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

  prepare_data(baro, odo, marg);
  nav_sins.run();
  navi2acs();
  navi2mavlink();

  debug2mavlink();

  dbg_out_fill(nav_sins.navi_data);
  dbg_out_append_log();

#else
  (void)abs_press;
  (void)odo;
  (void)marg;
#endif
}



