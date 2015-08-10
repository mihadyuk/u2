#include <memory> /* for placement new() */

#include "main.h"

#include "drivetrain.hpp"
#include "param_registry.hpp"
#include "min_max.hpp"
#include "mavlink_local.hpp"
#include "mav_logger.hpp"

using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define DRIVETRAIN_RUDDER_DEBUG       TRUE

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern MavLogger mav_logger;

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

static uint8_t newbuf[max_const(sizeof(Engine1ch), sizeof(Engine2ch))];

#if DRIVETRAIN_RUDDER_DEBUG
__CCM__ static mavlink_named_value_float_t mavlink_named_value_float_struct = {0, 0, "RUDDER"};
__CCM__ static mavMail named_value_mail;
#endif

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

static void drivetrain2mavlink(const DrivetrainImpact &impact) {

  mavlink_named_value_float_struct.value = impact.ch[IMPACT_RUD];
  mavlink_named_value_float_struct.time_boot_ms = TIME_BOOT_MS;

#if DRIVETRAIN_RUDDER_DEBUG
  if (named_value_mail.free()) {
    named_value_mail.fill(&mavlink_named_value_float_struct,
        MAV_COMP_ID_ALL, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT);
    mav_logger.write(&named_value_mail);
  }
#endif
}

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
    engine = new(newbuf) Engine1ch(pwm);
  else if (1 == *veh)
    engine = new(newbuf) Engine2ch(pwm);
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

  drivetrain2mavlink(impact);

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
