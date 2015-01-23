#include "stabilizer.hpp"
#include "main.h"

#include "param_registry.hpp"

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
void StabilizerRover::update_impl(float roll_trgt, float pitch_trgt, float speed_trgt){

  float roll_impact, pitch_impact, thrust_impact;

  roll_impact   = pid_ail.update(state_vector.roll  - roll_trgt,  state_vector.wx);
  pitch_impact  = pid_ele.update(state_vector.pitch - pitch_trgt, state_vector.wy);
  thrust_impact = pid_spd.update(state_vector.vair  - speed_trgt, 0);

  /* convert from ACS values to impact values */
  roll_impact   = -putinrange(roll_impact   * *ail_sens, -1, 1);
  pitch_impact  = -putinrange(pitch_impact  * *ele_sens, -1, 1);
  thrust_impact =  putinrange(thrust_impact * THRUST_SENS, *thrust_min, 1);

  /* set impact values */
//  impact.angle[SERVO_NUMBER_LEFT_AIL]  = roll_impact;
//  impact.angle[SERVO_NUMBER_RIGHT_AIL] = roll_impact;
//
//  impact.angle[SERVO_NUMBER_LEFT_ELE]  = pitch_impact;
//  impact.angle[SERVO_NUMBER_RIGHT_ELE] = pitch_impact;
//
//  impact.thrust = thrust_impact;
//
//  impact.angle[SERVO_NUMBER_LEFT_RUD]     = 0;
//  impact.angle[SERVO_NUMBER_RIGHT_RUD]    = 0;
//
//  impact.angle[SERVO_NUMBER_LEFT_FLAP]    = 0;
//  impact.angle[SERVO_NUMBER_RIGHT_FLAP]   = 0;
//
//  impact.angle[SERVO_NUMBER_OTHER_STRUT]  = 0;
//  impact.angle[SERVO_NUMBER_OTHER_BREAK]  = -0.98; // release break
//  impact.angle[SERVO_NUMBER_OTHER_CHUTE]  = 0;
}



/**
 *
 */
void StabilizerRover::dryRun(void){

//  pid_ail.reset();
//  pid_ele.reset();
//  pid_spd.dryRun(impact.thrust / THRUST_SENS);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
StabilizerRover::StabilizerRover(Drive::DrivetrainImpact &impact, const StateVector &state_vector) :
Stabilizer(impact, state_vector)
{
  return;
}

/**
 *
 */
void StabilizerRover::start_impl(void){

  param_registry.valueSearch("ACS_k_pitch",   &k_pitch);
  param_registry.valueSearch("ACS_pitch_bal", &pitch_bal);
  param_registry.valueSearch("ACS_ail_sens",  &ail_sens);
  param_registry.valueSearch("ACS_ele_sens",  &ele_sens);
  param_registry.valueSearch("ACS_loop_on",   &loop_on);
  param_registry.valueSearch("ACS_thrust_min",&thrust_min);
}

/**
 *
 */
void StabilizerRover::stop(void){
  ready = false;
}





