#include "main.h"
#include "servo_tree.hpp"
#include "putinrange.hpp"
#include "param_registry.hpp"

using namespace Drive;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define SRV_MIN 1000
#define SRV_MID 1500
#define SRV_MAX 2000

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
uint16_t float2pwm(float a, int min, int mid, int max) {
  uint16_t ret;

  if (a > 0)
    ret = mid + (max - mid) * a;
  else
    ret = mid + (mid - min) * a;

  return putinrange(ret, SRV_MIN, SRV_MAX);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
ServoTree::ServoTree(PWM &pwm) :
pwm(pwm) {
  return;
}

/**
 *
 */
void ServoTree::start(void) {
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

  tmp = float2pwm(impact.a[IMPACT_YAW], *rud_min, *rud_mid, *rud_max);
  pwm.update(tmp, PWM_CH_RUD);
}


