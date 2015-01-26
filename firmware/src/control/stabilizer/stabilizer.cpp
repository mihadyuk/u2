#include "main.h"
#include "param_registry.hpp"

#include "stabilizer/stabilizer.hpp"
#include "impact.hpp"

using namespace control;

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

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
Stabilizer::Stabilizer(Drivetrain &drivetrain) :
drivetrain(drivetrain),
ready(false)
{
  return;
}

/**
 *
 */
void Stabilizer::update(const FutabaData &futaba_data,
                        const TargetVector &trgt,
                        const StateVector &state,
                        float dT) {
  Impact impact;

  osalDbgCheck(ready);

//  impact.a[IMPACT_ROLL]  = pid_roll.update(state.roll   - trgt.roll,  state.wx);
//  impact.a[IMPACT_PITCH] = pid_pitch.update(state.pitch - trgt.pitch, state.wy);
//  impact.a[IMPACT_YAW]   = pid_yaw.update(state.yaw     - trgt.yaw,   state.wz);
//  impact.a[IMPACT_SPEED] = pid_speed.update(state.vair  - trgt.speed, 0);

  if (OVERRIDE_LEVEL_STABILIZER == futaba_data.override_level) {
    osalSysHalt("Unrealized");
  }
  else {
    impact.a[IMPACT_ROLL]  = pid_roll.update(state.roll, trgt.roll, dT);
    impact.a[IMPACT_PITCH] = pid_pitch.update(state.pitch, trgt.pitch, dT);
    impact.a[IMPACT_YAW]   = pid_yaw.update(state.yaw, trgt.yaw, dT);
    impact.a[IMPACT_SPEED] = pid_speed.update(state.vair, trgt.speed, dT);
  }

  this->drivetrain.update(futaba_data, impact);
}

/**
 *
 */
void Stabilizer::start(void) {

  float *p, *i, *d;

  param_registry.valueSearch("PID_roll_P", &p),
  param_registry.valueSearch("PID_roll_I", &i),
  param_registry.valueSearch("PID_roll_D", &d);
  pid_roll.start(p, i, d, nullptr, nullptr);

  param_registry.valueSearch("PID_pitch_P", &p),
  param_registry.valueSearch("PID_pitch_I", &i),
  param_registry.valueSearch("PID_pitch_D", &d);
  pid_pitch.start(p, i, d, nullptr, nullptr);

  param_registry.valueSearch("PID_yaw_P", &p),
  param_registry.valueSearch("PID_yaw_I", &i),
  param_registry.valueSearch("PID_yaw_D", &d);
  pid_yaw.start(p, i, d, nullptr, nullptr);

  param_registry.valueSearch("PID_speed_P", &p),
  param_registry.valueSearch("PID_speed_I", &i),
  param_registry.valueSearch("PID_speed_D", &d);
  pid_speed.start(p, i, d, nullptr, nullptr);

  drivetrain.start();

  ready = true;
}

/**
 *
 */
void Stabilizer::reset(void) {
  pid_roll.reset();
  pid_pitch.reset();
  pid_yaw.reset();
  pid_speed.reset();
}

/**
 *
 */
void Stabilizer::stop(void) {
  ready = false;
  drivetrain.stop();
}
