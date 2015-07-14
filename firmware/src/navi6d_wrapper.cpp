#pragma GCC optimize "-O2"
#pragma GCC optimize "-ffast-math"
#pragma GCC optimize "-funroll-loops"
#pragma GCC diagnostic ignored "-Wdouble-promotion"

#include <math.h>
#include "main.h"

#include "navigator_sins.hpp"
#include "kalman_flags.cpp" // dirty hack allowing to not add this file to the Makefile

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

#define KALMAN_DEBUG_LOG          FALSE

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_debug_t                 mavlink_out_debug_struct;
extern mavlink_debug_vect_t            mavlink_out_debug_vect_struct;
extern mavlink_highres_imu_t           mavlink_out_highres_imu_struct;

#if KALMAN_DEBUG_LOG
extern MavLogger mav_logger;
#endif

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

typedef double klmnfp;
#define KALMAN_STATE_SIZE         15
#define KALMAN_MEASUREMENT_SIZE   17

__CCM__ static NavigatorSins<klmnfp, KALMAN_STATE_SIZE, KALMAN_MEASUREMENT_SIZE> nav_sins;
__CCM__ static InitParams<klmnfp> init_params;
__CCM__ static CalibParams<klmnfp> calib_params;
__CCM__ static KalmanParams<klmnfp> kalman_params;
__CCM__ static RefParams<klmnfp> ref_params;

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
#if KALMAN_DEBUG_LOG
  if (dbg_in_mail.free()) {
    dbg_in_mail.fill(&dbg_in_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT);
    mav_logger.write(&dbg_in_mail);
  }
#endif
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

  dbg_out_struct.kalman_state_size  = KALMAN_STATE_SIZE;
  dbg_out_struct.kalman_meas_size   = KALMAN_MEASUREMENT_SIZE;

  dbg_out_struct.time_boot_ms = TIME_BOOT_MS;
}

/**
 *
 */
static void dbg_out_append_log(void) {
#if KALMAN_DEBUG_LOG
  if (dbg_out_mail.free()) {
    dbg_out_mail.fill(&dbg_out_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT);
    mav_logger.write(&dbg_out_mail);
  }
#endif
}

/**
 *
 */
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

/**
 *
 */
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

/**
 *
 */
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

/*
 * Some functions was moved to this file to reduce copypasta size
 * between test and main code
 */
#include "navi6d_common.cpp"

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

  gps.fresh = false;
  GNSS.subscribe(&gps);

  read_settings();
  restart_cache = *restart + 1; // enforce sins restart in first update call

  ready = true;
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
                           const marg_data_t &marg) {
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

  dbg_in_fill_gnss(this->gps);
  prepare_data_gnss(this->gps);
  dbg_in_fill_other(baro, odo, marg);
  dbg_in_append_log();
  prepare_data(baro, odo, marg);

  nav_sins.run();

  navi2acs();
  navi2mavlink();
  debug2mavlink();

  dbg_out_fill(nav_sins.navi_data);
  dbg_out_append_log();
}

