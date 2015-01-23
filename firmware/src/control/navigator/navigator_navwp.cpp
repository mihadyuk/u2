#include <math.h>
#include <time.h>

#include "main.h"
#include "navigator.hpp"
#include "coord_frame_conv.hpp"
#include "geometry.hpp"
#include "timekeeper.hpp"
#include "waypoint_db.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define GOVNOCODE_STABILIZE_TIMEOUT       S2ST(3)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern TimeKeeper time_keeper;

extern mavlink_nav_controller_output_t  mavlink_out_nav_controller_output_struct;
extern mavlink_griffon_target_vector_t  mavlink_out_griffon_target_vector_struct;

extern WpDB wpdb;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 *
 */
navigator_status_t Navigator::loop_navigate(void){
  return loop_navigate_global();
}

/**
 *
 */
navigator_status_t Navigator::loop_navigate_local(void){
  chDbgPanic("Unrealized");
  return NAVIGATOR_STATUS_DONE;
}

/**
 *
 */
navigator_status_t Navigator::loop_navigate_global(void){

  float pitch_trgt, roll_trgt;

  /* check reachablillity */
#if defined(ECEF_NAVIGATION)
  if (crsline_switch_ecef()){
    broadcast_mission_item_reached(mi.seq);
    what_to_do_here();
  }
#elif defined(SPHERE_NAVIGATION)
  if (crsline_switch_sphere()){
    broadcast_mission_item_reached(mi.seq);
    what_to_do_here();
  }
#else
#error "You must chose navigation method"
#endif

  mavlink_out_griffon_target_vector_struct.yaw = crsline_hdg;

  /* course channel */
  if (fabs(dZ) > *dZ_lim){
    float dHead;
    dHead = state_vector.yaw - crsline_hdg;
    if (dHead > (float)PI){
      dHead -= (float)PI2;
    }
    if (dHead < -(float)PI){
      dHead += (float)PI2;
    }

    // hysteresis
    if (fabs(dHead) > 177.0f * (float)DEG_TO_RAD) {
      dHead = (float)PI;
    }
    roll_trgt = *k_dZ * dZ + *k_dHead * dHead;
    roll_trgt = putinrange(roll_trgt, -*roll_lim, *roll_lim);
  }
  else{
    roll_trgt = 0;
  }

  /* height channel */
  pitch_trgt = *k_dH * dH + *k_dG * (state_vector.roll * state_vector.roll);
  pitch_trgt = putinrange(pitch_trgt, -*pitch_lim, *pitch_lim);
  pitch_trgt += *pitch_bal;

  /* FIXME: еще один ебучий говнохак для прибытия по расписанию */
  //float speed   = mi.param4;
  float speed     = GRIFFON_CRUISING_SPEED;

  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
  uint32_t temp   = *((uint32_t *)&mi.param4);
  #pragma GCC diagnostic pop

  if ((temp & 0x80000000UL) == 0) {
    /* fly by speed*/
    speed = (float)(temp & 0x7FFFFFFFUL) * 0.1f;
  }
  else {
    /* fly by time.*/
    uint32_t time_arrival = (uint32_t)(temp & 0x7FFFFFFF);
    uint32_t time = time_keeper.utc() / 1000000;
    speed = mavlink_out_nav_controller_output_struct.wp_dist / (time_arrival - time);
    speed = putinrange(speed, GRIFFON_CRUISING_SPEED, GRIFFON_MAX_SPEED);
  }
//  uint32_t time_arrival = *((uint32_t *)&mi.param4);
//  if (time_arrival > GRIFFON_MAX_SPEED){ /* значит вместо скорости тут время UTC */
//    uint32_t time_arrival = (time_t)mi.param4;
//    uint32_t time = time_keeper.utc() / 1000000;
//    speed = mavlink_out_nav_controller_output_struct.wp_dist / (time_arrival - time);
//    speed = putinrange(speed, GRIFFON_CRUISING_SPEED, GRIFFON_MAX_SPEED);
//  }

  /**/
  stabilizer.update(roll_trgt, pitch_trgt, speed);
  return NAVIGATOR_STATUS_DONE;
}

/**
 *
 */
navigator_status_t Navigator::EmergencyReturn(void){

  chSysLock();

  mi_home_start = mi; /* hack to fill all fields */

  mi_home_start.x = state_vector.lat;  //lat
  mi_home_start.y = state_vector.lon;  //lon
  mi_home_start.z = state_vector.alt_baro;  //alt
  mi_home_start.param4 = GRIFFON_CRUISING_SPEED; // just to be safer
  mi_home_start.command = MAV_CMD_NAV_WAYPOINT;
  mi_home_start.seq = 0;

  going_home = true;

  chSysUnlock();

  return NAVIGATOR_STATUS_DONE;
}

/**
 *
 */
navigator_status_t Navigator::UnMemorizeHome(void){
  chSysLock();
  mi_home_end.x = 0;
  mi_home_end.y = 0;
  chSysUnlock();

  return NAVIGATOR_STATUS_DONE;
}

/**
 *
 */
navigator_status_t Navigator::MemorizeHome(void){

  chSysLock();

  if ((mi_home_end.x == 0) && (mi_home_end.y == 0)){
    mi_home_end = mi;

    mi_home_end.x = state_vector.lat;  //lat
    mi_home_end.y = state_vector.lon;  //lon
    mi_home_end.z = state_vector.alt_baro;  //alt
    mi_home_end.param4 = GRIFFON_CRUISING_SPEED; // just to be safer
    mi_home_end.command = MAV_CMD_NAV_WAYPOINT;
    mi_home_end.seq = 1;
  }

  chSysUnlock();

  return NAVIGATOR_STATUS_DONE;
}

/**
 *
 */
static float height_sample = 250;
navigator_status_t Navigator::semiauto_govnocode(void){

  float pitch_trgt, roll_trgt, speed_trgt;
  float height_trgt;
  float dHead, dH_;

  /* mode init */
  if (0xFFFFFFFF == semiauto_first_call){
    semiauto_first_call = chTimeNow();
    height_sample = state_vector.alt_baro;
  }

  if ((chTimeNow() - semiauto_first_call) < GOVNOCODE_STABILIZE_TIMEOUT){
    roll_trgt = 0;
    dH_ = state_vector.alt_baro - height_sample;
    height_trgt = height_sample;
  }
  else{
    /* roll channel*/
    dHead = state_vector.yaw - semiauto_vector.hdg;
    roll_trgt = *k_dHead * dHead;
    roll_trgt = putinrange(roll_trgt, -*roll_lim, *roll_lim);

    //дельта по высоте (текущая - заданная с защитой от дурака)
    height_trgt = putinrange(semiauto_vector.alt, 200, 3000);
    dH_ = state_vector.alt_baro - height_trgt;
  }

  /* height channel */
  pitch_trgt = *k_dH * dH_ + *k_dG * (state_vector.roll * state_vector.roll);
  pitch_trgt = putinrange(pitch_trgt, -*pitch_lim, *pitch_lim);
  pitch_trgt += *pitch_bal;

  /* */
  speed_trgt = putinrange(semiauto_vector.air_speed, GRIFFON_CRUISING_SPEED, GRIFFON_MAX_SPEED);

  /* fill telemetry */
  mavlink_out_griffon_target_vector_struct.alt    = height_trgt;
  mavlink_out_griffon_target_vector_struct.speed  = speed_trgt;
  mavlink_out_griffon_target_vector_struct.yaw    = semiauto_vector.hdg;

  /* if connection lost is activated ignore semiauto command.*/
  if (this->isConnectionLost == true) {
    this->stabilizer.update(0, *this->pitch_bal, GRIFFON_CRUISING_SPEED);
    return NAVIGATOR_STATUS_DONE;
  }

  stabilizer.update(roll_trgt, pitch_trgt, speed_trgt);
  return NAVIGATOR_STATUS_DONE;
}

bool Navigator::IsEmergencyConnectionLost() {
  return this->isConnectionLost;
}
/**
 *
 */
bool Navigator::crsline_switch_sphere(void) {
  float target_heading;
  float target_distance;
  float xtd, atd;
  bool overshot;

  /* heading update */
  sphere.crosstrack(state_vector.lat, state_vector.lon, &xtd, &atd);
  sphere.course(state_vector.lat, state_vector.lon, &target_heading, &target_distance);
  overshot = sphere.isOvershot(state_vector.lat, state_vector.lon);

  /* HACK: I do not know why this -1 needed */
  dZ = -rad2m(xtd);
  if(isinf(dZ) || isnan(dZ))
    dZ = 0;

  /* height error (current - target) */
  dH = state_vector.alt_baro - mi.z;

  /* HACK: I do not know why this workaround needed */
  target_heading = wrap_2pi(-target_heading);

  /* update mavlink telemetry message with fresh parameters */
  mavlink_out_nav_controller_output_struct.nav_bearing = rad2deg(state_vector.yaw);
  mavlink_out_nav_controller_output_struct.target_bearing = rad2deg(target_heading);
  mavlink_out_nav_controller_output_struct.xtrack_error = dZ;
  mavlink_out_nav_controller_output_struct.alt_error = dH;
  mavlink_out_nav_controller_output_struct.wp_dist = rad2m(target_distance);

  if ((rad2m(target_distance) < mi.R) || (true == overshot))
    return true;
  else
    return false;
}

/**
 *
 */
bool Navigator::crsline_switch_ecef(void) {

  float rl[3]; // направляющий вектор ЛЗП (NED)
  float rc[3]; // направляющий вектор (NED)
  float wp1[3], wp2[3], cp[3]; // coordinates in WGS84
  float tmp;
  float d_crsln, dR;
  bool geo2lv_res1, geo2lv_res2;

  chSysLock();
  if (true == going_home){
    wp1[0] = mi_home_start.x;  //lat
    wp1[1] = mi_home_start.y;  //lon
    wp1[2] = mi_home_start.z;  //alt

    wp2[0] = mi_home_end.x;    //lat
    wp2[1] = mi_home_end.y;    //lon
    wp2[2] = mi_home_end.z;    //alt
  }
  else{
    wp1[0] = mi_prev.x;  //lat
    wp1[1] = mi_prev.y;  //lon
    wp1[2] = mi_prev.z;  //alt

    wp2[0] = mi.x;       //lat
    wp2[1] = mi.y;       //lon
    wp2[2] = mi.z;       //alt
  }
  chSysUnlock();

  cp[0] = state_vector.lat;
  cp[1] = state_vector.lon;
  cp[2] = state_vector.alt_baro;

  geo2lv_res1 = geodetic2lv<float>(rl, wp2, wp1); //NED
  geo2lv_res2 = geodetic2lv<float>(rc, cp, wp2);  //NED
  if ((CH_FAILED == geo2lv_res1) || (CH_FAILED == geo2lv_res2))
    return false;

  dH = cp[2] - wp2[2];        //дельта по высоте (текущая - заданная)

  //Длина текущей ЛЗП
  d_crsln = sqrt(rl[0]*rl[0] + rl[1]*rl[1]);

  /* update cross track error */
  dZ = (rl[0]*rc[1] - rl[1]*rc[0]) / d_crsln; // расстояние от текущей точки до ЛЗП, (векторное произведение в 2D)
                                              // "-" - точка лежит слева, если смотреть вдоль направляющего вектора текущей ЛЗП
                                              // "+" - точка лежит справа
  mavlink_out_nav_controller_output_struct.xtrack_error = dZ;
  if (dZ > *dZmax){
    dZ = *dZmax;
  }
  else if (dZ < -*dZmax){
    dZ = -*dZmax;
  }

  // курс текущей ЛЗП
  if ((rl[0] != 0) && (rl[1] != 0)){
    crsline_hdg = atan2(rl[1], rl[0]); // -180:180
    // 0:360
    if (crsline_hdg < 0)
      crsline_hdg += (float)PI2;
  }

  //радиус круга, в котором находится текущая точка
  dR = sqrt(rc[0]*rc[0] + rc[1]*rc[1]);

  /* update mavlink telemetry message with fresh parameters */
  mavlink_out_nav_controller_output_struct.nav_bearing = rad2deg(state_vector.yaw);
  mavlink_out_nav_controller_output_struct.target_bearing = rad2deg(crsline_hdg);
  mavlink_out_nav_controller_output_struct.alt_error = dH;
  mavlink_out_nav_controller_output_struct.wp_dist = dR;

  mavlink_out_griffon_target_vector_struct.wp_dist = dR;
  mavlink_out_griffon_target_vector_struct.xtrack_error = mavlink_out_nav_controller_output_struct.xtrack_error;

  // Критерий попадания в точку (ЛУР)
  if (dR < mi.R)
    return true;

  // критерий пересечения перпендикуляра к ЛЗП (скалярное произведение становится положительным)
  tmp = rl[0]*rc[0] + rl[1]*rc[1];
  if (tmp > 0)
    return true;

  return false; // default
}

navigator_status_t Navigator::EmergencyJump(uint16_t seq) {

  this->semLock.wait();
  navigator_status_t retval = this->_EmergencyJump(seq);
  this->semLock.signal();
  return retval;
}

bool Navigator::IsGoingHome() {
  return this->going_home;
}

/**
 *
 */
navigator_status_t Navigator::_EmergencyJump(uint16_t seq){
  bool load_status = CH_FAILED;
  acs_mission_item_t tmp;

  if (wpdb.loadCount() < seq){ // mission list overflow
    return NAVIGATOR_STATUS_ERROR;
  }
  else if (0 == seq){
    return NAVIGATOR_STATUS_ERROR; // jump to take off point forbidden
  }
  else{
    load_status = wpdb.load(&tmp, seq);
  }

  /**/
  if (CH_FAILED == load_status)
    return NAVIGATOR_STATUS_ERROR;
  else{
    chSysLock();
    mi = tmp;
    if (mi.command == MAV_CMD_NAV_LOITER_TURNS ||
        mi.command == MAV_CMD_NAV_LOITER_TIME ||
        mi.command == MAV_CMD_NAV_LOITER_UNLIM){
      this->state = NAVIGATOR_STATE_LOITER;
    }
    mi_prev.x = state_vector.lat;  //lat
    mi_prev.y = state_vector.lon;  //lon
    mi_prev.z = state_vector.alt_baro;  //alt
    chSysUnlock();

    broadcast_mission_current(mi.seq);
    sphere.updatePoints(mi_prev.x, mi_prev.y, mi.x, mi.y);
    return NAVIGATOR_STATUS_DONE;
  }

  chDbgPanic("Navigator::EmergencyJump. Logic broken");
  return NAVIGATOR_STATUS_DONE; /* warning suppressor */
}
