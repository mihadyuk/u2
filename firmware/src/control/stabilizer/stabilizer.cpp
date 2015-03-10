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
pid_speed(nullptr),
pid_roll(nullptr),
pid_pitch(nullptr),
pid_yaw(nullptr),
ready(false)
{
  return;
}

//  impact.a[IMPACT_ROLL]  = pid_roll.update(state.roll   - trgt.roll,  state.wx);
//  impact.a[IMPACT_PITCH] = pid_pitch.update(state.pitch - trgt.pitch, state.wy);
//  impact.a[IMPACT_YAW]   = pid_yaw.update(state.yaw     - trgt.yaw,   state.wz);
//  impact.a[IMPACT_SPEED] = pid_speed.update(state.vair  - trgt.speed, 0);

/**
 *
 */
void Stabilizer::update(const FutabaData &futaba_data,
                        const TargetAttitude &target_attitude,
                        const StateVector &state,
                        float dT) {
  Impact impact;
  const float *ta;

  osalDbgCheck(ready);

  if (OverrideLevel::attitude == futaba_data.level) {
    osalSysHalt("Unrealized/Untested");
    ta = futaba_data.attitude.a;
  }
  else {
    ta = target_attitude.a;
  }

  impact.a[IMPACT_CH_ROLL]  = pid_roll.update(state.roll,   ta[ATTITUDE_CH_ROLL],   dT);
  impact.a[IMPACT_CH_PITCH] = pid_pitch.update(state.pitch, ta[ATTITUDE_CH_PITCH],  dT);
  impact.a[IMPACT_CH_YAW]   = pid_yaw.update(state.yaw,     ta[ATTITUDE_CH_YAW],    dT);
  impact.a[IMPACT_CH_SPEED] = pid_speed.update(state.vair,  ta[ATTITUDE_CH_SPEED],  dT);

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
