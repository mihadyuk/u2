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
extern mavlink_highres_imu_t            mavlink_out_highres_imu_struct;

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
                   const baro_data_t &abs_press, const speedometer_data_t &speed) {

  sensor_data.r_sns[0][0] = deg2rad(gps_data.latitude);
  sensor_data.r_sns[1][0] = deg2rad(gps_data.longitude);
  sensor_data.r_sns[2][0] = gps_data.altitude;
  if ((el.getAndClearFlags() & EVMSK_GPS_FRESH_VALID) > 0) {
    osalDbgCheck((fabsf(gps_data.latitude) > 0.01) && (fabsf(gps_data.altitude) > 0.01));
    sensor_flags.sns_r_en = true;
    sensor_flags.sns_h_en = true;
  }

  sensor_data.v_sns[0][0] = gps_data.speed * cos(deg2rad(gps_data.course));
  sensor_data.v_sns[1][0] = gps_data.speed * sin(deg2rad(gps_data.course));
  sensor_data.v_sns[2][0] = 0;
  sensor_flags.sns_v_n_en = true;
  sensor_flags.sns_v_e_en = true;
  sensor_flags.sns_v_d_en = false;

  sensor_data.v_odo[0][0] = speed.speed;
  sensor_data.v_odo[1][0] = 0;
  sensor_data.v_odo[2][0] = 0;
  sensor_flags.odo_en = false;

  sensor_data.alt_b[0][0] = abs_press.alt;
  sensor_flags.alt_b_en = true;

  sensor_data.fb[0][0] = mavlink_out_highres_imu_struct.xacc;
  sensor_data.fb[1][0] = mavlink_out_highres_imu_struct.yacc;
  sensor_data.fb[2][0] = mavlink_out_highres_imu_struct.zacc;

  sensor_data.wb[0][0] = mavlink_out_highres_imu_struct.xgyro;
  sensor_data.wb[1][0] = mavlink_out_highres_imu_struct.ygyro;
  sensor_data.wb[2][0] = mavlink_out_highres_imu_struct.zgyro;

  sensor_data.mb[0][0] = mavlink_out_highres_imu_struct.xmag;
  sensor_data.mb[1][0] = mavlink_out_highres_imu_struct.ymag;
  sensor_data.mb[2][0] = mavlink_out_highres_imu_struct.zmag;
}

/**
 *
 */
void Navi6dWrapper::navi2acs(void) {

  const NaviData<double> &data = nav_sins.navi_data;

  acs_in.ch[ACS_INPUT_roll] = data.eu_nv[0][0];
  acs_in.ch[ACS_INPUT_pitch] = data.eu_nv[1][0];
  acs_in.ch[ACS_INPUT_yaw] = data.eu_nv[2][0];

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

  ref_params.eu_hb[0][0] = 3.14;
  ref_params.eu_hb[1][0] = 0;
  ref_params.eu_hb[2][0] = 0;
  for (size_t i = 0; i<3;i++){
     ref_params.eu_vh_base[i][0] = 0;
  }

  for (size_t i = 0; i<3; i++) {
    calib_params.ba_sat[i][0] = 0;
    calib_params.bw_sat[i][0] = 0;
  }
  calib_params.alpha = 0.02;

  //sensor_data.alt_b[0][0] = acs_in.ch[ACS_INPUT_alt_baro];
  for (size_t i = 0; i<9; i++) {
    kalman_params.R[i][0] = 1.0;
  }

  kalman_params.R[0][0] = 25;
  kalman_params.R[1][0] = 0.0001;
  kalman_params.R[2][0] = 0.01;

  for (size_t i = 0; i<10; i++) {
    kalman_params.Qm[i][0] = 1.0;
  }

  kalman_params.Qm[0][0] = 0.01;
  kalman_params.Qm[1][0] = 0.01;
  kalman_params.Qm[2][0] = 0.01;
  kalman_params.Qm[3][0] = 0.01;
  kalman_params.Qm[4][0] = 0.000001;
  kalman_params.Qm[5][0] = 0.000001;

  for (int i = 0; i<10;i++){
    init_params.Pi[i][0] = 0.0;
  }
  for (int i = 0; i<3;i++){
    init_params.Pi[i][0] = 0.0;
    init_params.r_init[i][0] = 0;
    init_params.v_init[i][0] = 0;
    init_params.eu_nv_init[i][0] = 0;
  }
  init_params.qnv_init[0][0] = 1;
  for (int i = 1; i<4;i++){
    init_params.qnv_init[i][0] = 0;
  }

  init_params.Pi[0][0] = 1; //r
  init_params.Pi[1][0] = 0.01; //v
  init_params.Pi[2][0] = 0.5; //tilt
  init_params.Pi[3][0] = 0.0; //head
  init_params.Pi[4][0] = 0.0000000001; //ba
  init_params.Pi[5][0] = 0.0000000001; //bw

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
              const baro_data_t &abs_press, const speedometer_data_t &speed) {

  prepare_data(gps_data, abs_press, speed);
  nav_sins.run(sensor_data, sensor_flags);
  navi2acs();
}



