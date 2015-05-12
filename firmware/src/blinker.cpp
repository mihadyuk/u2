#include <stdlib.h>

#include "main.h"
#include "pads.h"
#include "blinker.hpp"

/*
 * Blinker.
 * To use blinker you have to push pointer to int16_t array to the apropriate
 * mailbox.
 *
 * Array example:
 * static const int16_t myblink[5] = {100, -100, 100, -100, 0}
 * - numbers denotes duration of pulses in milliseconds
 * - negative values switching LED off, positive - on
 * - sequence must be terminated by zero
 * - total duration of sequence must be less or equal to 1 second
 */

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
typedef enum {
  BLINKER_WARNING,
  BLINKER_NORMAL,
}blinker_named_color_t;

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

Blinker blinker;

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
static chibios_rt::Mailbox<const int16_t *, 2> red_blink_mb;
static chibios_rt::Mailbox<const int16_t *, 2> blue_blink_mb;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

static void led_on(blinker_named_color_t color){
  switch(color){
  case(BLINKER_WARNING):
    warning_led_on();
    break;
  case(BLINKER_NORMAL):
    normal_led_on();
    break;
  }
}

static void led_off(blinker_named_color_t color){
  switch(color){
  case(BLINKER_WARNING):
    warning_led_off();
    break;
  case(BLINKER_NORMAL):
    normal_led_off();
    break;
  }
}

/**
 *
 */
static void blink_cycle(const int16_t *seq, blinker_named_color_t color){
  uint32_t i = 0;
  uint32_t sum = 0;

  /* check for probable overflow */
  i = 0;
  while (0 != seq[i]){
    sum += abs(seq[i]);
    i++;
  }
  osalDbgAssert((sum <= 1000), "Blinker sequence too long");

  /* main blinker */
  i = 0;
  while (0 != seq[i]){
    if (seq[i] > 0)
      led_on(color);
    else
      led_off(color);
    chThdSleepMilliseconds(abs(seq[i]));
    i++;
  }
}

/**
 * Blinker thread for red LED.
 */
static THD_WORKING_AREA(RedBlinkThreadWA, 192);
static THD_FUNCTION(RedBlinkThread, arg) {
  chRegSetThreadName("WarningBlink");
  (void)arg;
  const int16_t *rx;
  msg_t status = MSG_RESET;

  while(!chThdShouldTerminateX()){
    status = red_blink_mb.fetch(&rx, MS2ST(100));

    if (MSG_OK == status){
      blink_cycle(rx, BLINKER_WARNING);
    }
  }

  chThdExit(MSG_OK);
}

/**
 * Blinker thread for red LED.
 */
static THD_WORKING_AREA(BlueBlinkThreadWA, 192);
static THD_FUNCTION(BlueBlinkThread, arg) {
  chRegSetThreadName("NormalBlink");
  (void)arg;
  const int16_t *rx;
  msg_t status = MSG_RESET;

  while(!chThdShouldTerminateX()){
    status = blue_blink_mb.fetch(&rx, MS2ST(100));

    if (MSG_OK == status){
      blink_cycle(rx, BLINKER_NORMAL);
    }
  }

  chThdExit(MSG_OK);
}

/**
 *
 */
#define PAUSE() chThdSleepMilliseconds(100)
static THD_FUNCTION(BootBlinkThread, arg) {
  chRegSetThreadName("BootBlink");
  (void)arg;

  while (!chThdShouldTerminateX()) {
    red_led_on();
    blue_led_off();
    PAUSE();

    red_led_off();
    blue_led_on();
    PAUSE();

    blue_led_off();
    red_led_off();
    PAUSE();
  }

  blue_led_off();
  red_led_off();
  chThdExit(MSG_OK);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
Blinker::Blinker(void) {

}

/**
 *
 */
void Blinker::start(void){

  if (nullptr != this->redworker) {
    chThdTerminate(this->redworker);
    chThdWait(this->redworker);
  }

  this->redworker = chThdCreateStatic(RedBlinkThreadWA,
          sizeof(RedBlinkThreadWA),
          NORMALPRIO - 10,
          RedBlinkThread,
          NULL);

  this->blueworker = chThdCreateStatic(BlueBlinkThreadWA,
          sizeof(BlueBlinkThreadWA),
          NORMALPRIO - 10,
          BlueBlinkThread,
          NULL);

  ready = true;
}

/**
 *
 */
void Blinker::stop(void){

  ready = false;

  chThdTerminate(redworker);
  chThdTerminate(blueworker);

  chThdWait(redworker);
  chThdWait(blueworker);

  redworker = NULL;
  blueworker = NULL;
}

/**
 *
 */
void Blinker::error_post(const int16_t *array){
  if (true == ready)
    red_blink_mb.post(array, TIME_IMMEDIATE);
}

/**
 *
 */
void Blinker::normal_post(const int16_t *array){
  if (true == ready)
    blue_blink_mb.post(array, TIME_IMMEDIATE);
}

/**
 *
 */
void Blinker::bootIndication(void) {

  this->redworker = chThdCreateStatic(RedBlinkThreadWA,
          sizeof(RedBlinkThreadWA),
          NORMALPRIO - 10,
          BootBlinkThread,
          NULL);

  osalDbgCheck(nullptr != this->redworker);
}
