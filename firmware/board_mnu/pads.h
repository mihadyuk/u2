#ifndef PADS_H_
#define PADS_H_

#include "board.h"

static inline void nvram_power_on(void)       {palClearPad(GPIOH,   GPIOH_NVRAM_PWR);}
static inline void nvram_power_off(void)      {palSetPad(GPIOH,     GPIOH_NVRAM_PWR);}

static inline void mpu6050_power_on(void)     {palClearPad(GPIOH,   GPIOH_MPU6050_PWR);}
static inline void mpu6050_power_off(void)    {palSetPad(GPIOH,     GPIOH_MPU6050_PWR);}

static inline void red_led_on(void)           {palSetPad(GPIOI,     GPIOI_LED_R);}
static inline void red_led_off(void)          {palClearPad(GPIOI,   GPIOI_LED_R);}
static inline void red_led_toggle(void)       {palTogglePad(GPIOI,  GPIOI_LED_R);}

static inline void green_led_on(void)         {palSetPad(GPIOI,     GPIOI_LED_G);}
static inline void green_led_off(void)        {palClearPad(GPIOI,   GPIOI_LED_G);}
static inline void green_led_toggle(void)     {palTogglePad(GPIOI,  GPIOI_LED_G);}

static inline void orange_led_on(void)        {palSetPad(GPIOI,     GPIOI_LED_O);}
static inline void orange_led_off(void)       {palClearPad(GPIOI,   GPIOI_LED_O);}
static inline void orange_led_toggle(void)    {palTogglePad(GPIOI,  GPIOI_LED_O);}

#endif /* PADS_H_ */
