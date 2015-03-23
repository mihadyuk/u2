#include "main.h"

#include "servo_tree.hpp"
#include "param_registry.hpp"
#include "float2pwm.hpp"

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
ServoTree::ServoTree(PWM &pwm) : pwm(pwm) {
  return;
}

/**
 *
 */
void ServoTree::start(void) {

  param_registry.valueSearch("SRV_ail_min", &ail_min);
  param_registry.valueSearch("SRV_ail_mid", &ail_mid);
  param_registry.valueSearch("SRV_ail_max", &ail_max);

  param_registry.valueSearch("SRV_ele_min", &ele_min);
  param_registry.valueSearch("SRV_ele_mid", &ele_mid);
  param_registry.valueSearch("SRV_ele_max", &ele_max);

  param_registry.valueSearch("SRV_rud_min", &rud_min);
  param_registry.valueSearch("SRV_rud_mid", &rud_mid);
  param_registry.valueSearch("SRV_rud_max", &rud_max);

  ready = true;
}


/**
 *
 */
void ServoTree::stop(void) {
  ready = false;
}

/**
 *
 */
void ServoTree::update(const DrivetrainImpact &impact) {
  uint16_t tmp;

  osalDbgCheck(ready);

  tmp = float2pwm(impact.ch[PID_CHAIN_AIL], *ail_min, *ail_mid, *ail_max);
  pwm.update(tmp, PWM_CH_AIL);

  tmp = float2pwm(impact.ch[PID_CHAIN_ELE], *ele_min, *ele_mid, *ele_max);
  pwm.update(tmp, PWM_CH_ELE);

  tmp = float2pwm(impact.ch[PID_CHAIN_RUD], *rud_min, *rud_mid, *rud_max);
  pwm.update(tmp, PWM_CH_RUD);
}


