#include "main.h"

#include "exti_local.hpp"
#include "adis.hpp"
#include "mpu6050.hpp"
#include "lsm303_mag.hpp"
#include "time_keeper.hpp"
#include "gnss_receiver.hpp"
#include "bmp085.hpp"

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

ExtiPnc Exti;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/**
 *
 */
static void rtcalarm_cb(EXTDriver *extp, expchannel_t channel){
  (void)extp;
  (void)channel;
//  if (RTCD1.id_rtc->ISR | RTC_ISR_ALRBF)
//    RTCD1.id_rtc->ISR &= ~RTC_ISR_ALRBF;
//  if (RTCD1.id_rtc->ISR | RTC_ISR_ALRAF)
//    RTCD1.id_rtc->ISR &= ~RTC_ISR_ALRAF;
}

/**
 *
 */
static void rtcwakeup_cb(EXTDriver *extp, expchannel_t channel){
  (void)extp;
  (void)channel;
//  if (RTCD1.id_rtc->ISR | RTC_ISR_WUTF)
//    RTCD1.id_rtc->ISR &= ~RTC_ISR_WUTF;
}

/**
 *
 */
static void pps_cb(EXTDriver *extp, expchannel_t channel){
  (void)extp;
  (void)channel;

  osalSysLockFromISR();
  TimeKeeper::PPS_ISR_I();
  GNSS_PPS_ISR_I();
  osalSysUnlockFromISR();
}

/**
 *
 */
static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_RISING_EDGE  | EXT_MODE_GPIOA, pps_cb},
    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOE, Adis::extiISR},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE  | EXT_MODE_GPIOE, MPU6050::extiISR},
    {EXT_CH_MODE_RISING_EDGE  | EXT_MODE_GPIOE, BMP085::extiISR},//4
    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOE, LSM303_mag::extiISR},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},//8
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},//12
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},//16
    {EXT_CH_MODE_RISING_EDGE, rtcalarm_cb},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE, rtcwakeup_cb},
  }
};

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
ExtiPnc::ExtiPnc(void):
ready(false)
{
  return;
}

/**
 *
 */
void ExtiPnc::start(void){
  extStart(&EXTD1, &extcfg);
  ready = true;
}

/**
 *
 */
void ExtiPnc::stop(void){
  extStop(&EXTD1);
  ready = false;
}

/**
 * Enables interrupts from ADIS
 */
void ExtiPnc::adis(bool flag){
  osalDbgCheck(ready);
  if (flag)
    extChannelEnable(&EXTD1, GPIOE_ADIS_INT);
  else
    extChannelDisable(&EXTD1, GPIOE_ADIS_INT);
}

/**
 * Enables interrupts from MPU
 */
void ExtiPnc::mpu6050(bool flag){
  osalDbgCheck(ready);
  if (flag)
    extChannelEnable(&EXTD1, GPIOE_MPU9150_INT);
  else
    extChannelDisable(&EXTD1, GPIOE_MPU9150_INT);
}

/**
 *
 */
void ExtiPnc::lsm303(bool flag){
  osalDbgCheck(ready);
  if (flag)
    extChannelEnable(&EXTD1, GPIOE_MAG_INT);
  else
    extChannelDisable(&EXTD1, GPIOE_MAG_INT);
}

/**
 *
 */
void ExtiPnc::pps(bool flag){
  osalDbgCheck(ready);
  if (flag)
    extChannelEnable(&EXTD1, GPIOA_GPS_PPS);
  else
    extChannelDisable(&EXTD1, GPIOA_GPS_PPS);
}

/**
 *
 */
void ExtiPnc::bmp085(bool flag) {
  osalDbgCheck(ready);
  if (flag)
    extChannelEnable(&EXTD1, GPIOE_BMP085_EOC);
  else
    extChannelDisable(&EXTD1, GPIOE_BMP085_EOC);
}
