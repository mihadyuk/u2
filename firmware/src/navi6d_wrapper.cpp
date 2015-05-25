#pragma GCC optimize "-funroll-loops"
#pragma GCC optimize "-O2"

#include <math.h>
#include "main.h"
#include "math_f.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include "navigator_sins.hpp"
#pragma GCC diagnostic pop

#include "navi6d_wrapper.hpp"
#include "mavlink_local.hpp"
#include "acs_input.hpp"
#include "eb500.hpp"
#include "geometry.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

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

__CCM__ static NavigatorSins<double, 9, 6> nav_sins;

__CCM__ static InitParams<double> init_params;
__CCM__ static CalibParams<double> calib_params;
__CCM__ static KalmanParams<double> kalman_params;
__CCM__ static RefParams<double> ref_params;

__CCM__ static SensorData<double> sensor_data;
__CCM__ static KalmanFlags sensor_flags;

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

/*
 *
 */
void Navi6dWrapper::prepare_data(const gps_data_t &gps_data,
                                 const baro_data_t &abs_press,
                                 const speedometer_data_t &speed,
                                 const marg_data_t &marg)
{
  sensor_flags.sns_r_en = false;
  sensor_flags.sns_h_en = false;

  sensor_flags.sns_v_n_en = false;
  sensor_flags.sns_v_e_en = false;
  sensor_flags.sns_v_d_en = false;
  sensor_flags.alt_b_en = false;

  if ((el.getAndClearFlags() & EVMSK_GPS_FRESH_VALID) > 0) {
    osalDbgCheck((fabsf(gps_data.latitude) > 0.01) && (fabsf(gps_data.altitude) > 0.01));
    sensor_data.r_sns[0][0] = deg2rad(gps_data.latitude);
    sensor_data.r_sns[1][0] = deg2rad(gps_data.longitude);
    sensor_data.r_sns[2][0] = gps_data.altitude;
    sensor_flags.sns_r_en = true;
    sensor_flags.sns_h_en = true;

    sensor_data.v_sns[0][0] = gps_data.speed * cos(deg2rad(gps_data.course));
    sensor_data.v_sns[1][0] = gps_data.speed * sin(deg2rad(gps_data.course));
    sensor_data.v_sns[2][0] = 0;
    sensor_flags.sns_v_n_en = true;
    sensor_flags.sns_v_e_en = true;
    sensor_flags.sns_v_d_en = false;
    sensor_flags.alt_b_en = false;
  }

  sensor_data.v_odo[0][0] = speed.speed;
  sensor_data.v_odo[1][0] = 0;
  sensor_data.v_odo[2][0] = 0;
  sensor_flags.odo_en = false;

  sensor_data.alt_b[0][0] = abs_press.alt;
  sensor_flags.baro_fix_en = false;

  for(size_t i=0; i<3; i++) {
    sensor_data.fb[i][0] = marg.acc[i];
    sensor_data.wb[i][0] = marg.gyr[i];
    sensor_data.mb[i][0] = marg.mag[i];
  }
}

/**
 *
 */
void Navi6dWrapper::navi2acs(void) {

  const NaviData<double> &data = nav_sins.navi_data;

  acs_in.ch[ACS_INPUT_roll]   = data.eu_nv[0][0];
  acs_in.ch[ACS_INPUT_pitch]  = data.eu_nv[1][0];
  acs_in.ch[ACS_INPUT_yaw]    = data.eu_nv[2][0];

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

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
Navi6dWrapper::Navi6dWrapper(ACSInput &acs_in) : acs_in(acs_in) {
  return;
}

/**
 *
 */
void Navi6dWrapper::start(float dT) {

  event_gps.registerMask(&el, EVMSK_GPS_FRESH_VALID);

  ref_params.eu_hb[0][0] = 0;
  ref_params.eu_hb[1][0] = 0;
  ref_params.eu_hb[2][0] = 0;

  ref_params.eu_vh_base[0][0] = M_PI;
  ref_params.eu_vh_base[1][0] = 0;
  ref_params.eu_vh_base[2][0] = 0;

  for (size_t i = 0; i<3; i++) {
    calib_params.ba_sat[i][0] = 0;
    calib_params.bw_sat[i][0] = 0;
  }
  calib_params.alpha = 0.02;

  kalman_params.sigma_R[0][0] = 5;
  kalman_params.sigma_R[1][0] = 7;
  kalman_params.sigma_R[2][0] = 0.01;

  kalman_params.sigma_Qm[0][0] = 0.01;
  kalman_params.sigma_Qm[1][0] = 0.01;
  kalman_params.sigma_Qm[2][0] = 0.000001;
  kalman_params.sigma_Qm[3][0] = 0.000001;
  kalman_params.sigma_Qm[4][0] = 0.000001;
  kalman_params.sigma_Qm[5][0] = 0.000001;

  for (int i = 0; i<3;i++){
    init_params.sigma_Pi[i][0] = 0.0;
    init_params.r_init[i][0] = 0;
    init_params.v_init[i][0] = 0;
    init_params.eu_nv_init[i][0] = 0;
  }
  init_params.qnv_init[0][0] = 1;
  for (int i = 1; i<4;i++){
    init_params.qnv_init[i][0] = 0;
  }

  init_params.sigma_Pi[0][0] = 1; //r
  init_params.sigma_Pi[1][0] = 0.01; //v
  init_params.sigma_Pi[2][0] = 0.5; //tilt
  init_params.sigma_Pi[3][0] = 20.0; //head
  init_params.sigma_Pi[4][0] = 0.0000000001; //ba
  init_params.sigma_Pi[5][0] = 0.0000000001; //bw

  init_params.dT = dT;
  init_params.rst_cnt = 250;
  init_params.alpha = 0.03;

  nav_sins.set_init_params(init_params);
  nav_sins.set_calib_params(calib_params);
  nav_sins.set_kalman_params(kalman_params);
  nav_sins.set_ref_params(ref_params);

  nav_sins.command_executor(1);
}

/**
 *
 */
void Navi6dWrapper::stop(void) {

  event_gps.unregister(&el);
}

/**
 *
 */
void Navi6dWrapper::update(const gps_data_t &gps_data,
                           const baro_data_t &abs_press,
                           const speedometer_data_t &speed,
                           const marg_data_t &marg)
{
  prepare_data(gps_data, abs_press, speed, marg);
  nav_sins.run(sensor_data, sensor_flags);
  navi2acs();
}



