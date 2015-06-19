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

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
#if ! FAKE_SINS
__CCM__ static NavigatorSins<double, 15, 13> nav_sins;
__CCM__ static InitParams<double> init_params;
__CCM__ static CalibParams<double> calib_params;
__CCM__ static KalmanParams<double> kalman_params;
__CCM__ static RefParams<double> ref_params;
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
void Navi6dWrapper::prepare_gnss(const speedometer_data_t &speed) {
#if ! FAKE_SINS
  if ((*gnss_enable == 1) && (gnss_data.fresh) && (gnss_data.fix > 0)) {
    nav_sins.sensor_data.r_sns[0][0] = deg2rad(gnss_data.latitude);
    nav_sins.sensor_data.r_sns[1][0] = deg2rad(gnss_data.longitude);
    nav_sins.sensor_data.r_sns[2][0] = gnss_data.altitude;
    nav_sins.sensor_flags.sns_r_en = true;
    nav_sins.sensor_flags.sns_h_en = true;

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
  else {
    nav_sins.sensor_data.v_odo[0][0] = speed.speed;
    nav_sins.sensor_data.v_odo[1][0] = 0;
    nav_sins.sensor_data.v_odo[2][0] = 0;
  }

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
                                 const speedometer_data_t &speed,
                                 const marg_data_t &marg)
{
  prepare_gnss(speed);

  if (*odo_enable == 1) {
    nav_sins.sensor_flags.odo_en = true;
    nav_sins.sensor_flags.nonhol_y_en = true;
    nav_sins.sensor_flags.nonhol_z_en = true;
  }

  if (*baro_enable == 1) {
    nav_sins.sensor_flags.alt_b_en = true;
    nav_sins.sensor_data.alt_b[0][0] = abs_press.alt;
    nav_sins.sensor_flags.baro_fix_en = true;
  }

  for(size_t i=0; i<3; i++) {
    nav_sins.sensor_data.fb[i][0] = marg.acc[i];
    nav_sins.sensor_data.wb[i][0] = marg.gyr[i];
    nav_sins.sensor_data.mb[i][0] = marg.mag[i];
  }
}
#else
void Navi6dWrapper::prepare_data(const baro_data_t &abs_press,
                                 const speedometer_data_t &speed,
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
void Navi6dWrapper::navi2acs(void) {

  const NaviData<double> &data = nav_sins.navi_data;

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

  //mavlink_out_debug_struct.value = data.status & (1UL << 5UL);
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

  param_registry.valueSearch("SINS_gnss_enable",&gnss_enable);
  param_registry.valueSearch("SINS_odo_enable", &odo_enable);
  param_registry.valueSearch("SINS_baro_enable",&baro_enable);

  param_registry.valueSearch("SINS_sigma_R0",   &sigma_R0);
  param_registry.valueSearch("SINS_sigma_R1",   &sigma_R1);
  param_registry.valueSearch("SINS_sigma_R2",   &sigma_R2);
  param_registry.valueSearch("SINS_sigma_R3",   &sigma_R3);
  param_registry.valueSearch("SINS_sigma_R4",   &sigma_R4);
  param_registry.valueSearch("SINS_sigma_R5",   &sigma_R5);
  param_registry.valueSearch("SINS_sigma_R6",   &sigma_R6);

  param_registry.valueSearch("SINS_sigma_Qm0",  &sigma_Qm0);
  param_registry.valueSearch("SINS_sigma_Qm1",  &sigma_Qm1);
  param_registry.valueSearch("SINS_sigma_Qm2",  &sigma_Qm2);
  param_registry.valueSearch("SINS_sigma_Qm3",  &sigma_Qm3);
  param_registry.valueSearch("SINS_sigma_Qm4",  &sigma_Qm4);
  param_registry.valueSearch("SINS_sigma_Qm5",  &sigma_Qm5);

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

  init_params.sigma_Pi[0][0] = 200; //initial position STD (m)
  init_params.sigma_Pi[3][0] = M_PI; //initial heading STD (rad)

  kalman_params.sigma_R[0][0] = *sigma_R0; //ne_sns
  kalman_params.sigma_R[1][0] = *sigma_R1; //d_sns
  kalman_params.sigma_R[2][0] = *sigma_R2; //v_n_sns
  kalman_params.sigma_R[3][0] = *sigma_R3; //odo
  kalman_params.sigma_R[4][0] = *sigma_R4; //nonhol
  kalman_params.sigma_R[5][0] = *sigma_R5; //baro
  kalman_params.sigma_R[6][0] = *sigma_R6; //mag

#if MAIN_SENSOR_ADIS
  kalman_params.sigma_Qm[0][0] = *sigma_Qm0; //acc
  kalman_params.sigma_Qm[1][0] = *sigma_Qm1; //gyr
  kalman_params.sigma_Qm[2][0] = *sigma_Qm2; //acc_x
  kalman_params.sigma_Qm[3][0] = *sigma_Qm3; //acc_y
  kalman_params.sigma_Qm[4][0] = *sigma_Qm4; //acc_z
  kalman_params.sigma_Qm[5][0] = *sigma_Qm5; //gyr_bias
#else // MPU6050
  kalman_params.sigma_Qm[0][0] = 0.01; //acc
  kalman_params.sigma_Qm[1][0] = 0.01; //gyr
  kalman_params.sigma_Qm[2][0] = 0.0001; //acc_x
  kalman_params.sigma_Qm[3][0] = 0.0001; //acc_y
  kalman_params.sigma_Qm[4][0] = 0.0001; //acc_z
  kalman_params.sigma_Qm[5][0] = 0.0005; //gyr_bias
#endif

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
                           const speedometer_data_t &speed,
                           const marg_data_t &marg)
{
#if ! FAKE_SINS
  osalDbgCheck(ready);

//  ref_params.glrt_acc_sigma = *acc_sigma;
//  ref_params.glrt_gyr_sigma = *gyr_sigma;
//  ref_params.glrt_n = *samples;
//  ref_params.glrt_gamma = *gamma;
  nav_sins.set_ref_params(ref_params);

  prepare_data(abs_press, speed, marg);
  nav_sins.run();
  navi2acs();

  mavlink_out_debug_vect_struct.time_usec = TimeKeeper::utc();
//  mavlink_out_debug_vect_struct.x = nav_sins.navi_data.wb_c[0][0];
//  mavlink_out_debug_vect_struct.y = nav_sins.navi_data.wb_c[1][0];
//  mavlink_out_debug_vect_struct.z = nav_sins.navi_data.wb_c[2][0];
  mavlink_out_debug_vect_struct.x = nav_sins.navi_data.a_bias[0][0];
  mavlink_out_debug_vect_struct.y = nav_sins.navi_data.a_bias[1][0];
  mavlink_out_debug_vect_struct.z = nav_sins.navi_data.a_bias[2][0];

//  if (nav_sins.glrt_det.status)
//    red_led_on();
//  else
//    red_led_off();

#else
  (void)abs_press;
  (void)speed;
  (void)marg;
#endif
}



