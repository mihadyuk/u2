#include "main.h"

#include "stabilizer_plane.hpp"
#include "param_registry.hpp"
#include "servo_numbers.h"
#include "acs_telemetry.hpp"

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
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
static const float THRUST_SENS = 0.01; // from percents to 0..1
static const float THRUST_MID_POINT = 50; // mid point for PID in percents
static const float RUD_LIM = 0.9;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 * @brief   It's a magic. Do not ask how it works and why it works.
 */
void StabilizerPlane::feng_shui(float *thrust_impact){
  float true_pitch = state_vector.pitch - *pitch_bal;

  if (true_pitch > 0)
    *thrust_impact += *k_pitch * true_pitch;
}

/**
 *
 */
void StabilizerPlane::update_impl(float roll_trgt, float pitch_trgt, float speed_trgt){

  float roll_impact, pitch_impact, yaw_impact, thrust_impact;

  yaw_impact    = pid_rud.update(state_vector.free_ay_body, state_vector.wz);
//  yaw_impact    = pid_rud.update(-6, state_vector.wz);
  roll_impact   = pid_ail.update(state_vector.roll  - roll_trgt,  state_vector.wx);
  pitch_impact  = pid_ele.update(state_vector.pitch - pitch_trgt, state_vector.wy);
  thrust_impact = pid_spd.update(state_vector.vair  - speed_trgt, 0);
  thrust_impact += THRUST_MID_POINT;

  #if defined(BOARD_NTLAB_GRIFFON_ACS)
  stabilizer2telemetry(pid_spd, pid_ail, pid_ele, pid_rud, roll_impact, pitch_impact);
  #endif

  /* convert from ACS values to normalized impact values.
     WARNING! yaw_impact не нуджается в конверсии, потому что его
     индентефекацией занимались без сахаруков и сивашков */
  roll_impact   = -putinrange(roll_impact   * *ail_sens,    -*ail_lim,    *ail_lim);
  pitch_impact  = -putinrange(pitch_impact  * *ele_sens,    -*ele_lim,    *ele_lim);
  thrust_impact =  putinrange(thrust_impact * THRUST_SENS,  *thrust_min,  1);
  yaw_impact    =  putinrange(yaw_impact, -RUD_LIM, RUD_LIM);

  /* magic */
  feng_shui(&thrust_impact);

  /* set impact values */
  impact.angle[SERVO_NUMBER_LEFT_AIL]  = roll_impact;
  impact.angle[SERVO_NUMBER_RIGHT_AIL] = roll_impact;

  impact.angle[SERVO_NUMBER_LEFT_ELE]  = pitch_impact;
  impact.angle[SERVO_NUMBER_RIGHT_ELE] = pitch_impact;

  impact.thrust = thrust_impact;

  impact.angle[SERVO_NUMBER_LEFT_RUD]     = yaw_impact;
  impact.angle[SERVO_NUMBER_RIGHT_RUD]    = yaw_impact;

  impact.angle[SERVO_NUMBER_LEFT_FLAP]    = 1;
  impact.angle[SERVO_NUMBER_RIGHT_FLAP]   = 1;

  impact.angle[SERVO_NUMBER_OTHER_STRUT]  = 0;
  impact.angle[SERVO_NUMBER_OTHER_BREAK]  = -0.98; // release break
  impact.angle[SERVO_NUMBER_OTHER_CHUTE]  = 0;
}

/**
 *
 */
void StabilizerPlane::reset_impl(void){

  pid_ail.reset();
  pid_ele.reset();
  pid_spd.reset();
  pid_rud.reset();
}

/**
 *
 */
void StabilizerPlane::dryRun(void){

  pid_ail.reset();
  pid_ele.reset();
  pid_rud.reset();
  pid_spd.dryRun(impact.thrust / THRUST_SENS);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
StabilizerPlane::StabilizerPlane(Impact &impact, const StateVector &state_vector) :
Stabilizer(impact, state_vector)
{
  ready = false;
}

/**
 *
 */
void StabilizerPlane::start(void){

  float *iMin, *iMax, *p, *i, *d;

  param_registry.valueSearch("ACS_spd_iMin",  &iMin),
  param_registry.valueSearch("ACS_spd_iMax",  &iMax),
  param_registry.valueSearch("ACS_spd_pGain", &p),
  param_registry.valueSearch("ACS_spd_iGain", &i),
  param_registry.valueSearch("ACS_spd_dGain", &d);
  pid_spd.start(iMin, iMax, p, i, d);

  param_registry.valueSearch("ACS_ail_iMin",  &iMin),
  param_registry.valueSearch("ACS_ail_iMax",  &iMax),
  param_registry.valueSearch("ACS_ail_pGain", &p),
  param_registry.valueSearch("ACS_ail_iGain", &i),
  param_registry.valueSearch("ACS_ail_dGain", &d);
  pid_ail.start(iMin, iMax, p, i, d);

  param_registry.valueSearch("ACS_ele_iMin",  &iMin),
  param_registry.valueSearch("ACS_ele_iMax",  &iMax),
  param_registry.valueSearch("ACS_ele_pGain", &p),
  param_registry.valueSearch("ACS_ele_iGain", &i),
  param_registry.valueSearch("ACS_ele_dGain", &d);
  pid_ele.start(iMin, iMax, p, i, d);

  param_registry.valueSearch("ACS_rud_iMin",  &iMin),
  param_registry.valueSearch("ACS_rud_iMax",  &iMax),
  param_registry.valueSearch("ACS_rud_pGain", &p),
  param_registry.valueSearch("ACS_rud_iGain", &i),
  param_registry.valueSearch("ACS_rud_dGain", &d);
  pid_rud.start(iMin, iMax, p, i, d);

  param_registry.valueSearch("ACS_k_pitch",   &k_pitch);
  param_registry.valueSearch("ACS_ail_sens",  &ail_sens);
  param_registry.valueSearch("ACS_ele_sens",  &ele_sens);
  param_registry.valueSearch("ACS_thrust_min",&thrust_min);
  param_registry.valueSearch("ACS_pitch_bal", &pitch_bal);
  param_registry.valueSearch("ACS_ele_lim",   &ele_lim);
  param_registry.valueSearch("ACS_ail_lim",   &ail_lim);

  ready = true;
}

/**
 *
 */
void StabilizerPlane::stop(void){
  ready = false;
}





