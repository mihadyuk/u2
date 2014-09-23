#include "main.h"

#include "exti_local.hpp"
#include "adis.hpp"
#include "marg_worker.hpp"

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
static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOE, Adis::extiISR},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOE, MPU6050ISR},
    {EXT_CH_MODE_DISABLED, NULL},//4
    {EXT_CH_MODE_DISABLED, NULL},
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

//static const EXTConfig extcfg = {
//  {
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOE, TimeKeeper::normal_pps_isr},     //4
//    {EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOE, imcuc_data_received_cb},
//    {EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOD, nand_ready_cb},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL}, //8
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},//12
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},//16
//    {EXT_CH_MODE_RISING_EDGE, rtcalarm_cb},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_RISING_EDGE, rtcwakeup_cb},
//  }
//};

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
  osalDbgCheck(ready == true);
  if (flag)
    extChannelEnable(&EXTD1, GPIOE_ADIS_INT);
  else
    extChannelDisable(&EXTD1, GPIOE_ADIS_INT);
}

/**
 * Enables interrupts from MPU
 */
void ExtiPnc::mpu6050(bool flag){
  osalDbgCheck(ready == true);
  if (flag)
    extChannelEnable(&EXTD1, GPIOE_MPU9150_INT);
  else
    extChannelDisable(&EXTD1, GPIOE_MPU9150_INT);
}
