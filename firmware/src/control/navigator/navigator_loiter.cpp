#include "main.h"

#include "navigator.hpp"
#include "mavdbg.hpp"
#include "kvand_circle.hpp"

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

extern mavlink_nav_controller_output_t  mavlink_out_nav_controller_output_struct;
extern mavlink_griffon_target_vector_t  mavlink_out_griffon_target_vector_struct;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */


#define NO_LOITERING    -666
static volatile float circles_dbg = NO_LOITERING; // for debug only
extern mavlink_griffon_surface_angles_t mavlink_out_griffon_surface_angles_struct;




/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static KvandCircle<float> kvand_circle;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 * @brief   Loitering path built from waypoints.
 */
navigator_status_t Navigator::loiter_engine_point(void){

  bool ret = !STD_MANEUR_END;

  /* entry point */
  std_maneur.init(mi);

  if (false == std_maneur_active){
    if (STD_MANEUR_END == std_maneur.next(mi_prev) || STD_MANEUR_END == std_maneur.next(mi)){
      this->state = NAVIGATOR_STATE_LOAD_MISSION_ITEM;
      return NAVIGATOR_STATUS_DONE;
    }
    else{
      std_maneur_active = true;
      return NAVIGATOR_STATUS_DONE;
    }
  }

  /* normal activity */
#if defined(ECEF_NAVIGATION)
  if (crsline_switch_ecef()){
    mi_prev = mi;
    ret = std_maneur.next(mi);
  }
#elif defined(SPHERE_NAVIGATION)
  if (crsline_switch_sphere()){
    mi_prev = mi;
    ret = std_maneur.next(mi);
  }
#else
#error "You must chose navigation method"
#endif

  if (STD_MANEUR_END == ret){
    std_maneur_active = false;
    this->state = NAVIGATOR_STATE_LOAD_MISSION_ITEM;
  }
  return NAVIGATOR_STATUS_DONE;
}

/**
 * @brief   Loitering procedure based on circle formulae.
 */
float Navigator::loiter_engine_curve(void){

  float ret;

  float rc[3]; // координаты центра окружности WGS84
  rc[0] = mi.x;       //lat
  rc[1] = mi.y;       //lon
  rc[2] = mi.z;       //alt

  float r[3]; // координаты трактора WGS84
  r[0] = state_vector.lat;
  r[1] = state_vector.lon;
  r[2] = state_vector.alt_baro;

  ret = kvand_circle.update(this->dZ, this->crsline_hdg, r, mi.R, rc);



  mavlink_out_griffon_surface_angles_struct.l_flap = mi.R;




  mavlink_out_nav_controller_output_struct.xtrack_error = dZ;
  mavlink_out_griffon_target_vector_struct.yaw = crsline_hdg;

  /* а теперь будем обмазываться несвежей копипастой */
  float pitch_trgt, roll_trgt;

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
  dH = r[2] - rc[2]; //дельта по высоте (текущая - заданная)
  pitch_trgt = *k_dH * dH + *k_dG * (state_vector.roll * state_vector.roll);
  pitch_trgt = putinrange(pitch_trgt, -*pitch_lim, *pitch_lim);
  pitch_trgt += *pitch_bal;

  /**/
  float speed = mi.param4;
  stabilizer.update(roll_trgt, pitch_trgt, speed);

  /* update mavlink telemetry message with fresh parameters */
  mavlink_out_nav_controller_output_struct.nav_bearing = rad2deg(state_vector.yaw);
  mavlink_out_nav_controller_output_struct.target_bearing = rad2deg(crsline_hdg);
  mavlink_out_nav_controller_output_struct.alt_error = dH;
  mavlink_out_griffon_target_vector_struct.xtrack_error = mavlink_out_nav_controller_output_struct.xtrack_error;

  return ret;
}

/**
 * @brief   Convert turns to time
 */
//static systime_t loop_param2st(float turns, float R, float speed) {
//  return S2ST(roundf((static_cast<float>(PI2) * R * turns) / speed));
//}

/**
 *
 */
navigator_status_t Navigator::loop_loiter_turns(void){

  if (false == std_maneur_active){
    loiter_circles = mi.param2 - 0.33333333f;
    std_maneur_active = true;
  }

  circles_dbg = loiter_engine_curve();

  /* temporal debug code. Move it after usage to acs_telemetry.cpp */
  mavlink_out_griffon_surface_angles_struct.r_rud = circles_dbg;

  if (circles_dbg >= loiter_circles) {
    this->state = NAVIGATOR_STATE_LOAD_MISSION_ITEM; /* loitering completed */
    mi.seq = mi_prev.seq;
    std_maneur_active = false;
    circles_dbg = NO_LOITERING;
    mavlink_out_griffon_surface_angles_struct.r_rud = circles_dbg;
    mavlink_out_griffon_surface_angles_struct.l_flap = NO_LOITERING;
  }

  return NAVIGATOR_STATUS_DONE;
}

/**
 *
 */
navigator_status_t Navigator::loop_loiter_time(void){

  systime_t now, timeout;

  if (false == std_maneur_active){
    loiter_start_time = 0;
    std_maneur_active = true;
  }

  if (0 == loiter_start_time){
    circles_dbg = loiter_engine_curve();
    if (circles_dbg > 0)
      loiter_start_time = chTimeNow(); /* start time counting only after circle touch */
    return NAVIGATOR_STATUS_DONE;
  }

  now = chTimeNow();
  timeout = S2ST(roundf(mi.param2));
  if ((now - loiter_start_time) < timeout){
    circles_dbg = loiter_engine_curve();
  }
  else{
    this->state = NAVIGATOR_STATE_LOAD_MISSION_ITEM; /* loitering completed */
    mi.seq = mi_prev.seq;
    std_maneur_active = false;
    circles_dbg = NO_LOITERING;
    mavlink_out_griffon_surface_angles_struct.l_flap = NO_LOITERING;
  }

  return NAVIGATOR_STATUS_DONE;
}

/**
 *
 */
navigator_status_t Navigator::loop_loiter_unlim(void){
  // unrealized yet
  stabilizer.update(0.25, *pitch_bal, GRIFFON_CRUISING_SPEED);
  return NAVIGATOR_STATUS_ERROR;
}

/**
 *
 */
navigator_status_t Navigator::EmergencyLoiter(float height_trgt){

  float dH = state_vector.alt_baro - height_trgt;
  float pitch_trgt;

  pitch_trgt = *k_dH * dH + *k_dG * (state_vector.roll * state_vector.roll);
  pitch_trgt = putinrange(pitch_trgt, -*pitch_lim, *pitch_lim);
  pitch_trgt += *pitch_bal;

  /**/
  stabilizer.update(0.25, pitch_trgt, GRIFFON_CRUISING_SPEED);
  return NAVIGATOR_STATUS_DONE;
}

/**
 *
 */
navigator_status_t Navigator::loop_loiter(void){

  /* stupidity protection */
  mi.param4 = putinrange(mi.param4, GRIFFON_CRUISING_SPEED, GRIFFON_MAX_SPEED);

  switch(mi.command){
  case MAV_CMD_NAV_LOITER_TURNS:
    return loop_loiter_turns();
    break;

  case MAV_CMD_NAV_LOITER_TIME:
    return loop_loiter_time();
    break;

  case MAV_CMD_NAV_LOITER_UNLIM:
    return loop_loiter_unlim();
    break;

  default:
    return NAVIGATOR_STATUS_ERROR;
    break;
  }

  return NAVIGATOR_STATUS_ERROR;
}

/**
 *
 */
navigator_status_t Navigator::EmergencyConnectionLost(void){
  /* connection lost state is actual when the UAV in semiauto mode.*/
  this->isConnectionLost = true;
  return NAVIGATOR_STATUS_DONE;
}

navigator_status_t Navigator::EmergencyResetConnectionLost(void) {
  this->isConnectionLost = false;
  return NAVIGATOR_STATUS_DONE;
}

#if 0
/**
 * @brief   Special kind of loiter
 */
navigator_status_t Navigator::loop_connection_lost(void){
#if 1
  this->stabilizer.update(0, *pitch_bal, GRIFFON_CRUISING_SPEED);
  return NAVIGATOR_STATUS_DONE;
#else
  if ((chTimeNow() - connection_lost_timestamp) > S2ST(60)){
    /* time is out. Construct navigation path to home and go home */
    navigator_status_t ret = NAVIGATOR_STATUS_ERROR;
    ret = this->EmergencyReturn();
    state = NAVIGATOR_STATE_NAVIGATE_TO_WAYPOINT;
    return ret;
  }
  else {
    this->stabilizer.update(0, *pitch_bal, GRIFFON_CRUISING_SPEED);
    return NAVIGATOR_STATUS_DONE;
  }
#endif
}
#endif

