#include "main.h"
#include "putinrange.hpp"
#include "array_len.hpp"
#include "geometry.hpp"
#include "param_registry.hpp"

#include <control/futaba/futaba.hpp>

using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
/**
 * @brief     Routing tables.
 * @details   Maps receiver's PWM channels to abstract channels from
 *            higher levels.
 */
struct route_table_impact_t {
  uint8_t roll    = 0;
  uint8_t pitch   = 1;
  uint8_t yaw     = 2;
  uint8_t speed   = 3;
};

struct route_table_direction_t {
  uint8_t course  = 0;
  uint8_t height  = 1;
  uint8_t speed   = 3;
};

struct route_table_attitude_t {
  uint8_t roll    = 0;
  uint8_t pitch   = 1;
  uint8_t yaw     = 2;
  uint8_t speed   = 3;
};

/**
 * @brief   Manual switch interpretation
 */
static const OverrideLevel switch_pos[] = {
    OverrideLevel::none,
    OverrideLevel::impact,
    OverrideLevel::direction
};

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
static const route_table_impact_t     route_impact;
static const route_table_direction_t  route_direction;
static const route_table_attitude_t   route_attitude;

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
static float pwm_normalize(uint16_t v) {
  const float shift = 1500;
  const float scale = 500;
  float ret;

  ret = ((float)v - shift) / scale;
  return putinrange(ret, -1, 1);
}

/**
 *
 */
static void pwm2direction(TargetDirection &dir, const uint16_t *pwm) {
  osalSysHalt("Unrealized/Untested");
  dir.a[DIRECTION_CH_COURSE] = pwm[route_direction.course] / 1000.0f;
}

/**
 *
 */
static void pwm2impact(Impact &impact, const uint16_t *pwm) {
  osalSysHalt("Unrealized/Untested");
  impact.a[IMPACT_CH_ROLL]  = pwm_normalize(pwm[route_impact.roll]);
  impact.a[IMPACT_CH_PITCH] = pwm_normalize(pwm[route_impact.pitch]);
  impact.a[IMPACT_CH_YAW]   = pwm_normalize(pwm[route_impact.yaw]);
  impact.a[IMPACT_CH_SPEED] = pwm_normalize(pwm[route_impact.speed]);
}

/**
 *
 */
static void pwm2attitude(TargetAttitude &att, const uint16_t *pwm) {

  float tmp;
  osalSysHalt("Unrealized/Untested");
  tmp = pwm_normalize(pwm[route_attitude.roll]);
  tmp = putinrange(tmp, deg2rad(-30), deg2rad(30));
  att.a[ATTITUDE_CH_ROLL]  = tmp;
}

/**
 *
 */
static void pwm2pwm(PwmVector &pwm_out, const uint16_t *pwm) {
  osalSysHalt("Unrealized/Untested");
  for (size_t i=0; i<ArrayLen(pwm_out.pwm); i++) {
    pwm_out.pwm[i] = pwm[i];
  }
}

/**
 *
 */
static bool is_data_fresh(receiver_data_t &recv) {

  if ((recv.status & RECEIVER_STATUS_CONN_LOST) != RECEIVER_STATUS_CONN_LOST)
    return true;
  else
    return false;
}

/**
 *
 */
msg_t process_pwm(FutabaData &result, const Receiver &receiver) {
  msg_t ret = MSG_TIMEOUT;
  receiver_data_t recv;

  receiver.update(recv);

  if (is_data_fresh(recv)) {
    pwm2direction(result.direction, recv.pwm);
    pwm2attitude(result.attitude, recv.pwm);
    pwm2impact(result.impact, recv.pwm);
    pwm2pwm(result.pwm_vector, recv.pwm);
  }

  return ret;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void Futaba::start(void) {

  param_registry.valueSearch("RC_timeout", &timeout);

  receiver_mavlink.start(*timeout);
  receiver_rc.start(*timeout);
  receiver_synth.start(*timeout);
  ready = true;
}

/**
 *
 */
void Futaba::stop(void){
  ready = false;
  receiver_synth.stop();
  receiver_rc.stop();
  receiver_mavlink.stop();
}

/**
 *
 */
msg_t Futaba::update(FutabaData &result) {

  msg_t ret = MSG_TIMEOUT;

  osalDbgCheck(ready);

//  /* RC has priority over mavlink futaba */
//  ret = process_pwm(result, receiver_rc);
//  if (MSG_OK == ret) {
//    // TODO: set apropriate override flags here
//    //if manual_switch_RC
//    return ret;
//  }
//
//  /* */
//  ret = process_pwm(result, receiver_mavlink);
//  if (MSG_OK == ret) {
//    // TODO: set apropriate override flags here
//    //if manual_switch_Mavlink
//    return ret;
//  }

  /* */
  ret = process_pwm(result, receiver_synth);
  if (MSG_OK == ret) {
    // TODO: set apropriate override flags here
    //if manual_switch_Mavlink
    result.level = OverrideLevel::none;
    result.impact.mask = 0;
    result.pwm_vector.mask = 0;
    return ret;
  }

  return ret;
}

