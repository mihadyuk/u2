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
extern mavlink_debug_t                 mavlink_out_debug_struct;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

__CCM__ static NavigatorSins<double, 9, 13> nav_sins;

__CCM__ static InitParams<double> init_params;
__CCM__ static CalibParams<double> calib_params;
__CCM__ static KalmanParams<double> kalman_params;
__CCM__ static RefParams<double> ref_params;


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

  if ((el.getAndClearFlags() & EVMSK_GPS_FRESH_VALID) > 0) {
    osalDbgCheck((fabsf(gps_data.latitude) > 0.01) && (fabsf(gps_data.altitude) > 0.01));
    nav_sins.sensor_data.r_sns[0][0] = deg2rad(gps_data.latitude);
    nav_sins.sensor_data.r_sns[1][0] = deg2rad(gps_data.longitude);
    nav_sins.sensor_data.r_sns[2][0] = gps_data.altitude;
    nav_sins.sensor_flags.sns_r_en = true;
    nav_sins.sensor_flags.sns_h_en = true;

    nav_sins.sensor_data.v_sns[0][0] = gps_data.speed * cos(deg2rad(gps_data.course));
    nav_sins.sensor_data.v_sns[1][0] = gps_data.speed * sin(deg2rad(gps_data.course));
    nav_sins.sensor_data.v_sns[2][0] = 0;
    nav_sins.sensor_flags.sns_v_n_en = true;
    nav_sins.sensor_flags.sns_v_e_en = true;
  }
  else {
    nav_sins.sensor_data.v_odo[0][0] = speed.speed;
    nav_sins.sensor_data.v_odo[1][0] = 0;
    nav_sins.sensor_data.v_odo[2][0] = 0;
    nav_sins.sensor_flags.odo_en = false;
    nav_sins.sensor_flags.nonhol_y_en = false;
    nav_sins.sensor_flags.nonhol_z_en = false;
  }

  nav_sins.sensor_flags.alt_b_en = false;

  nav_sins.sensor_data.alt_b[0][0] = abs_press.alt;
  nav_sins.sensor_flags.baro_fix_en = false;

  for(size_t i=0; i<3; i++) {
    nav_sins.sensor_data.fb[i][0] = marg.acc[i];
    nav_sins.sensor_data.wb[i][0] = marg.gyr[i];
    nav_sins.sensor_data.mb[i][0] = marg.mag[i];
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

  //mavlink_out_debug_struct.value = data.status & (1UL << 5UL);
  mavlink_out_debug_struct.value = data.mag_mod;
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


  ref_params.eu_vh_base[0][0] = 0;
  ref_params.eu_vh_base[1][0] = 0;
  ref_params.eu_vh_base[2][0] = 0;

  calib_params.bm[0][0] = 0.0338;
  calib_params.bm[1][0] = 0.03736;
  calib_params.bm[2][0] = -0.2661;

  calib_params.m_s[0][0] = 180.186;
  calib_params.m_s[1][0] =  178.2;
  calib_params.m_s[2][0] = 178.386;
  calib_params.m_no[0][0] = -1.444;
  calib_params.m_no[1][0] = -2.35684;
  calib_params.m_no[2][0] = -0.291037;

  kalman_params.sigma_R[0][0] = 5; //ne_sns
  kalman_params.sigma_R[1][0] = 7; //d_sns
  kalman_params.sigma_R[2][0] = 0.1; //v_n_sns
  kalman_params.sigma_R[3][0] = 0.5; //odo
  kalman_params.sigma_R[4][0] = 0.1; //nonhol
  kalman_params.sigma_R[5][0] = 0.3; //baro
  kalman_params.sigma_R[6][0] = 0.3; //mag

  kalman_params.sigma_Qm[0][0] = 0.005;
  kalman_params.sigma_Qm[1][0] = 0.005;
  kalman_params.sigma_Qm[2][0] = 0.000001;
  kalman_params.sigma_Qm[3][0] = 0.000001;
  kalman_params.sigma_Qm[4][0] = 0.000001;
  kalman_params.sigma_Qm[5][0] = 0.000001;

//  init_params.sigma_Pi[0][0] = 1; //r
//  init_params.sigma_Pi[1][0] = 0.01; //v
//  init_params.sigma_Pi[2][0] = 0.5; //tilt
  init_params.sigma_Pi[3][0] = 50.0; //head
//  init_params.sigma_Pi[4][0] = 0.0000000001; //ba
//  init_params.sigma_Pi[5][0] = 0.0000000001; //bw

  init_params.dT = dT;

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
  nav_sins.run();
  navi2acs();
}



