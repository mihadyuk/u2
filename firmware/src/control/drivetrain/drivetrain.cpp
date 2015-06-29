#include <memory> /* for placement new() */

#include "main.h"

#include "drivetrain.hpp"
#include "param_registry.hpp"
#include "min_max.hpp"

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

static uint8_t ebuf[max_const(sizeof(Engine1ch), sizeof(Engine2ch))];

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
Drivetrain::Drivetrain(void) :
engine(nullptr),
servo(pwm)
{
  return;
}

/**
 *
 */
void Drivetrain::start(void) {

  pwm.start();

  const uint32_t *veh;
  param_registry.valueSearch("SYS_vehicle_type",  &veh);
  if (0 == *veh)
    engine = new(ebuf) Engine1ch(pwm);
  else if (1 == *veh)
    engine = new(ebuf) Engine2ch(pwm);
  else
    osalSysHalt("Unhandled value");
  engine->start();

  servo.start();

  ready = true;
}

/**
 *
 */
void Drivetrain::stop(void) {
  ready = false;

  servo.stop();
  engine->stop();
  engine = nullptr;
  pwm.stop();
}

/**
 *
 */
msg_t Drivetrain::update(const DrivetrainImpact &impact) {

  osalDbgCheck(ready);

  servo.update(impact);
  engine->update(impact);

  return MSG_OK;
}

/**
 *
 */
uint32_t Drivetrain::capabilities(void) {
  uint32_t ret = 0;

  ret |= (1 << IMPACT_THR) | (1 << IMPACT_RUD);

  return ret;
}

/**
 *
 */
void Drivetrain::arm(void) {
  engine->arm();
}

/**
 *
 */
void Drivetrain::disarm(void) {
  engine->disarm();
}
