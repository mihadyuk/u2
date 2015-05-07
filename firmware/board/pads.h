#ifndef PADS_H_
#define PADS_H_

#include "board.h"

static inline void xbee_reset_assert(void)    {palClearPad(GPIOD, GPIOD_XBEE_RESET);}
static inline void xbee_reset_clear(void)     {palSetPad(GPIOD,   GPIOD_XBEE_RESET);}

static inline void adis_reset_assert(void)    {palClearPad(GPIOA, GPIOA_ADIS_NRST);}
static inline void adis_reset_clear(void)     {palSetPad(GPIOA,   GPIOA_ADIS_NRST);}

static inline void gps_power_on(void)         {palClearPad(GPIOE, GPIOE_GPS_ENABLE);}
static inline void gps_power_off(void)        {palSetPad(GPIOE,   GPIOE_GPS_ENABLE);}

static inline void pwr5v_power_on(void)       {palSetPad(GPIOD,   GPIOD_5V_ENABLE);}
static inline void pwr5v_power_off(void)      {palClearPad(GPIOD, GPIOD_5V_ENABLE);}

static inline void nvram_power_on(void)       {palClearPad(GPIOA, GPIOA_NVRAM_PWR);}
static inline void nvram_power_off(void)      {palSetPad(GPIOA,   GPIOA_NVRAM_PWR);}

static inline void microsd_power_on(void)     {palClearPad(GPIOE, GPIOE_SDIO_PWR);}
static inline void microsd_power_off(void)    {palSetPad(GPIOE,   GPIOE_SDIO_PWR);}

static inline void mpu6050_power_on(void)     {palSetPad(GPIOE,   GPIOE_MPU6050_PWR);}
static inline void mpu6050_power_off(void)    {palClearPad(GPIOE, GPIOE_MPU6050_PWR);}

static inline void blue_led_on(void)          {palClearPad(GPIOB, GPIOB_LED_B);}
static inline void blue_led_off(void)         {palSetPad(GPIOB,   GPIOB_LED_B);}
static inline void blue_led_toggle(void)      {palTogglePad(GPIOB,GPIOB_LED_B);}

static inline void red_led_on(void)           {palClearPad(GPIOB, GPIOB_LED_R);}
static inline void red_led_off(void)          {palSetPad(GPIOB,   GPIOB_LED_R);}
static inline void red_led_toggle(void)       {palTogglePad(GPIOB,GPIOB_LED_R);}

static inline void green_led_on(void)         {palClearPad(GPIOB, GPIOB_LED_G);}
static inline void green_led_off(void)        {palSetPad(GPIOB,   GPIOB_LED_G);}
static inline void green_led_toggle(void)     {palTogglePad(GPIOB,GPIOB_LED_G);}

static inline void normal_led_on(void)        {green_led_on();}
static inline void normal_led_off(void)       {green_led_off();}
static inline void warning_led_on(void)       {blue_led_on();}
static inline void warning_led_off(void)      {blue_led_off();}
static inline void error_led_on(void)         {red_led_on();}
static inline void error_led_off(void)        {red_led_off();}

#endif /* PADS_H_ */
