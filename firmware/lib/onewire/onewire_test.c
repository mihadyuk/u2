/*
    ChibiOS/RT - Copyright (C) 2014 Uladzimir Pylinsky aka barthess

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <string.h>

#include "onewire.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define ONEWIRE_MASTER_CHANNEL        2 /* this PWM channel drives bus */
#define ONEWIRE_SAMPLE_CHANNEL        3 /* this one generates interrupts when sampling needed */

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

#if ONEWIRE_USE_PARASITIC_POWER
static void strong_pullup_assert(void);
static void strong_pullup_release(void);
#endif

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static uint8_t testbuf[12];

static float temperature[3];

/*
 *
 */
static const OWConfig ow_cfg = {
  &PWMD4,
  ONEWIRE_MASTER_CHANNEL,
  ONEWIRE_SAMPLE_CHANNEL,
#if ONEWIRE_USE_PARASITIC_POWER
  strong_pullup_assert,
  strong_pullup_release
#endif
};

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

#if ONEWIRE_USE_PARASITIC_POWER
/**
 *
 */
static void strong_pullup_assert(void) {
  palSetPadMode(GPIOB, GPIOB_TACHOMETER, PAL_MODE_ALTERNATE(2) |
                  PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_PUDR_PULLUP);
}

/**
 *
 */
static void strong_pullup_release(void) {
  palSetPadMode(GPIOB, GPIOB_TACHOMETER, PAL_MODE_ALTERNATE(2) |
                PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUDR_PULLUP);
}
#endif /* ONEWIRE_PARASITIC_POWER_MODE */

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
#include "pads.h"
volatile bool presence;
/**
 *
 */
void onewireTest(void) {

  uint16_t tmp;
  uint8_t rombuf[24];
  size_t devices_on_bus = 0;
  size_t i = 0;

  onewireObjectInit(&OWD1);
  onewireStart(&OWD1, &ow_cfg);

#if SYNTH_SEARCH_TEST
  synthSearchRomTest(&OWD1);
#endif

  for (i=0; i<3; i++)
    temperature[i] = -666;

  while (true) {
    if (true == onewireReset(&OWD1)){

      memset(rombuf, 0x55, sizeof(rombuf));
      devices_on_bus = onewireSearchRom(&OWD1, rombuf, 3);
      osalDbgCheck(devices_on_bus <= 3);

      if (1 == devices_on_bus){
        /* test read rom command */
        presence = onewireReset(&OWD1);
        osalDbgCheck(true == presence);
        testbuf[0] = ONEWIRE_CMD_READ_ROM;
        onewireWrite(&OWD1, testbuf, 1, 0);
        onewireRead(&OWD1, testbuf, 8);
        osalDbgCheck(testbuf[7] == onewireCRC(testbuf, 7));
        osalDbgCheck(0 == memcmp(rombuf, testbuf, 8));
      }

      /* start temperature measurement on all connected devices at once */
      presence = onewireReset(&OWD1);
      osalDbgCheck(true == presence);
      testbuf[0] = ONEWIRE_CMD_SKIP_ROM;
      testbuf[1] = ONEWIRE_CMD_CONVERT_TEMP;

#if ONEWIRE_USE_PARASITIC_POWER
      onewireWrite(&OWD1, testbuf, 2, MS2ST(750));
#else
      onewireWrite(&OWD1, testbuf, 2, 0);
      /* poll bus waiting ready signal from all connected devices */
      testbuf[0] = 0;
      while (testbuf[0] == 0){
        osalThreadSleepMilliseconds(50);
        onewireRead(&OWD1, testbuf, 1);
      }
#endif

      for (i=0; i<devices_on_bus; i++) {
        /* read temperature device by device from their scratchpads */
        presence = onewireReset(&OWD1);
        osalDbgCheck(true == presence);

        testbuf[0] = ONEWIRE_CMD_MATCH_ROM;
        memcpy(&testbuf[1], &rombuf[i*8], 8);
        testbuf[9] = ONEWIRE_CMD_READ_SCRATCHPAD;
        onewireWrite(&OWD1, testbuf, 10, 0);

        red_led_on();
        onewireRead(&OWD1, testbuf, 9);
        red_led_off();

        osalDbgCheck(testbuf[8] == onewireCRC(testbuf, 8));
        tmp = 0;
        tmp |= (testbuf[1] << 8) | testbuf[0];
        temperature[i] = tmp * 0.0625;
      }
    }
    else {
      osalSysHalt("");
    }

    //palTogglePad(GPIOD, GPIOD_LED4);
    osalThreadSleep(1);
  }

  onewireStop(&OWD1);
}
