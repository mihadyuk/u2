#ifndef PADS_H_
#define PADS_H_

#include "board.h"

static inline void xbee_reset_assert(void)    {;}
static inline void xbee_reset_clear(void)     {;}
static inline void gps_power_on(void)         {;}
static inline void gps_power_off(void)        {;}
static inline void pwr5v_power_on(void)       {;}
static inline void pwr5v_power_off(void)      {;}

static inline void nvram_power_on(void)       {palClearPad(GPIOH,   GPIOH_NVRAM_PWR);}
static inline void nvram_power_off(void)      {palSetPad(GPIOH,     GPIOH_NVRAM_PWR);}

static inline void adis_reset_assert(void)    {;}
static inline void adis_reset_clear(void)     {;}

static inline void mpu6050_power_on(void)     {palClearPad(GPIOH,   GPIOH_MPU6050_PWR);}
static inline void mpu6050_power_off(void)    {palSetPad(GPIOH,     GPIOH_MPU6050_PWR);}

static inline void microsd_power_on(void)    {palSetPad(GPIOG,   GPIOG_MMC_NRST);}
static inline void microsd_power_off(void)   {palClearPad(GPIOG, GPIOG_MMC_NRST);}

static inline void red_led_on(void)           {palSetPad(GPIOI,     GPIOI_LED_R);}
static inline void red_led_off(void)          {palClearPad(GPIOI,   GPIOI_LED_R);}
static inline void red_led_toggle(void)       {palTogglePad(GPIOI,  GPIOI_LED_R);}

static inline void green_led_on(void)         {palSetPad(GPIOI,     GPIOI_LED_G);}
static inline void green_led_off(void)        {palClearPad(GPIOI,   GPIOI_LED_G);}
static inline void green_led_toggle(void)     {palTogglePad(GPIOI,  GPIOI_LED_G);}

static inline void orange_led_on(void)        {palSetPad(GPIOI,     GPIOI_LED_O);}
static inline void orange_led_off(void)       {palClearPad(GPIOI,   GPIOI_LED_O);}
static inline void orange_led_toggle(void)    {palTogglePad(GPIOI,  GPIOI_LED_O);}

static inline void normal_led_on(void)        {green_led_on();}
static inline void normal_led_off(void)       {green_led_off();}
static inline void normal_led_toggle(void)    {green_led_toggle();}

static inline void warning_led_on(void)       {orange_led_on();}
static inline void warning_led_off(void)      {orange_led_off();}
static inline void warning_led_toggle(void)   {orange_led_toggle();}

static inline void error_led_on(void)         {red_led_on();}
static inline void error_led_off(void)        {red_led_off();}
static inline void error_led_toggle(void)     {red_led_toggle();}

#endif /* PADS_H_ */
