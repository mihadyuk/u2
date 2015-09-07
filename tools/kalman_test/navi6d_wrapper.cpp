//#pragma GCC optimize "-O2"
//#pragma GCC optimize "-ffast-math"
//#pragma GCC optimize "-funroll-loops"
//#pragma GCC diagnostic ignored "-Wdouble-promotion"

#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "../../firmware/lib/navi6d/navigator_sins.hpp"
#include "../../firmware/lib/navi6d/kalman_flags.cpp" // dirty hack allowing to not add this file to the Makefile

#include "../../firmware/lib/mavlink/C/lapwing/mavlink.h"
#include "../../firmware/lib/uav_utils/geometry.hpp"
#include "navi6d_wrapper.hpp"

#include "param_registry.hpp"

using namespace std;

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

extern ParamRegistry param_registry;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

#if !defined(KALMAN_STATE_SIZE)
#define KALMAN_STATE_SIZE           21
#endif

#if !defined(KALMAN_MEASUREMENT_SIZE)
#define KALMAN_MEASUREMENT_SIZE     17
#endif

typedef double klmnfp;

static NavigatorSins<klmnfp, KALMAN_STATE_SIZE, KALMAN_MEASUREMENT_SIZE> nav_sins;
static InitParams<klmnfp> init_params;
static CalibParams<klmnfp> calib_params;
static KalmanParams<klmnfp> kalman_params;
static RefParams<klmnfp> ref_params;

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
static bool check_result(double r1, double r2, double tolerance) {

  if (std::isinf(r1) || std::isnan(r1))
    return false;

  if (std::isinf(r2) || std::isnan(r2))
    return false;

  if (std::abs(r1 - r2) > tolerance)
    return false;

  return true;
}

/**
 *
 */
static const double coordinate_tolerance = 5 / RAD_TO_M;
static const double height_tolerance = 10;
static const double ahrs_tolerance = deg2rad(5.0);
static double drop_seconds = 2 * 60;
static size_t total_run = 0;
static void print_failed_message(const NaviData<klmnfp> &data,
                                 const mavlink_navi6d_debug_output_t &ref) {
  cout << "failed on " << total_run << " iteration" << endl;
  cout << "ref.lat: " << ref.lat << ", calculated.lat: " << data.r[0][0]
      << ", tolerance: " << coordinate_tolerance << endl;
  cout << "ref.lon: " << ref.lon << ", calculated.lon: " << data.r[1][0]
      << ", tolerance: " << coordinate_tolerance << endl;
  cout << "ref.alt: " << ref.alt << ", calculated.alt: " << data.r[2][0]
      << ", tolerance: " << height_tolerance << endl;

  cout << "ref.roll: " << ref.roll << ", calculated.roll: " << data.eu_nv[0][0]
      << ", tolerance: " << ahrs_tolerance << endl;
  cout << "ref.pitch: " << ref.pitch << ", calculated.pitch: " << data.eu_nv[1][0]
      << ", tolerance: " << ahrs_tolerance << endl;
}

/**
 *
 */
static bool dbg_out_verify(const NaviData<klmnfp> &data,
                           const mavlink_navi6d_debug_output_t &ref,\
                           double dT) {

  total_run++;
  if (drop_seconds > 0) {
    drop_seconds -= dT;
  }
  else {
    if (! check_result(ref.lat, data.r[0][0], coordinate_tolerance)   ||
        ! check_result(ref.lon, data.r[1][0], coordinate_tolerance)   ||
        ! check_result(ref.alt, data.r[2][0], height_tolerance)       ||

        ! check_result(ref.roll,  data.eu_nv[0][0], ahrs_tolerance)   ||
        ! check_result(ref.pitch, data.eu_nv[1][0], ahrs_tolerance)) {
      print_failed_message(data, ref);
      throw std::exception();
    }
  }

  return true;
}

/*
 * Some functions was moved to this file to reduce copypasta size
 * between test and main code
 */
#include "../../firmware/src/navi6d_common.cpp"

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
    nav_sins.params.init_params.dT = marg.dT;
    nav_sins.params.init_params.rst_dT = 0.5;
  }

  /* restart sins if requested */
  if (*restart != restart_cache) {
    sins_cold_start();
    restart_cache = *restart;
  }

  prepare_data_gnss(gps);
  prepare_data(baro, odo, marg);
  nav_sins.run();

  dbg_out_verify(nav_sins.navi_data, ref, test.marg_dt);
}



