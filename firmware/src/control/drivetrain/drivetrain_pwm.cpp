#include <cstring>

#include "main.h"
#include "drivetrain_pwm.hpp"
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

static uint16_t pwm1[4] = {0, 0, 0, 0};
static uint16_t pwm4[4] = {0, 0, 0, 0};

/**
 * reload fresh values from RAM to timers
 */
static void pwm_cb(PWMDriver *pwmp) {
  size_t i = 0;
  uint16_t *val;

  if (&PWMD1 == pwmp)
    val = pwm1;
  else if (&PWMD4 == pwmp)
    val = pwm4;
  else {
    val = NULL;
    osalSysHalt("Unhandled case");
  }

  osalSysLockFromISR();
  for (i=0; i<4; i++)
    pwmEnableChannelI(pwmp, i, val[i]);
  osalSysUnlockFromISR();
}

/**
 *
 */
static const PWMConfig pwm_default_cfg = {
  DRIVETRAIN_PWM_CLK,
  DRIVETRAIN_PWM_PERIOD,
  pwm_cb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  0,
  0
};


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
PWM::PWM(void) {
  return;
}

/**
 *
 */
void PWM::start(void) {

  if (ready)
    return;

  pwmStart(&PWMD1, &pwm_default_cfg);
  pwmEnablePeriodicNotification(&PWMD1);

  /* pause between starts needs for spread PWM ISRs through time */
  osalThreadSleepMicroseconds(DRIVETRAIN_PWM_PERIOD / 2);

  pwmStart(&PWMD4, &pwm_default_cfg);
  pwmEnablePeriodicNotification(&PWMD4);

  ready = true;
}

/**
 *
 */
void PWM::stop(void) {

  if (!ready)
    return;

  ready = false;

  pwmStop(&PWMD4);
  pwmStop(&PWMD1);
}

/**
 *
 */
void PWM::update(uint16_t pwm, size_t channel) {

  osalDbgCheck(ready && (channel < DRIVETRAIN_PWM_CHANNELS));

  if (channel < 4)
    pwm1[channel] = pwm;
  else if (channel < 8)
    pwm4[channel - 4] = pwm;
  else
    return;
}

/**
 *
 */
void PWM::futaba_override(const PwmVector &override) {
  size_t i;

  osalDbgCheck(ready);

  for (i=0; i<4; i++)
    pwm1[i] = override.pwm[i];
  for (; i<8; i++)
    pwm1[i-4] = override.pwm[i];
}

