#include "main.h"
#include "pads.h"

#include "adis.hpp"
#include "exti_local.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define ADIS_START_TIME_MS        560
#define ADIS_SPI                  SPID1

/* some address definitions */
#define DIAG_STS          0x0A
#define GLOB_CMD          0x02
#define PROD_ID           0x7E
#define SYS_E_FLAG        0x08

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
Adis adis;

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
 * Software delay for this stupid sensor - 1.5uS
 */
static const size_t ADIS_NSS_DELAY_US = (STM32_SYSCLK + STM32_SYSCLK / 2) / 1000000;

static const uint16_t ADIS_SAMPLE_RATE_DIV = 24;
static const uint16_t ADIS_INTERNAL_SAMPLE_RATE = 2460;

static const uint16_t supported_models[] = {16480};
static const size_t supported_models_cnt = sizeof(supported_models) / sizeof(supported_models[0]);

/*
 * Maximum speed SPI configuration (84MHz/8, CPHA=1, CPOL=1, 16bit, MSb first).
 */
static const SPIConfig spicfg = {
  NULL,
  GPIOA,
  GPIOA_ADIS_NSS,
  SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_DFF
};

chibios_rt::BinarySemaphore adis_sem(true);

static const uint8_t request[] = {
    0x08, // sys error flags
    0x0E, // temp
    0x10, // GYR
    0x12,
    0x14,
    0x16,
    0x18,
    0x1A,
    0x1C, // ACC
    0x1E,
    0x20,
    0x22,
    0x24,
    0x26,
    0x28, // mag
    0x2A,
    0x2C,
    0x2E,
    0x30,
    0x60, // Q0 (c11)
    0x62, // Q1 (c12)
    0x64, // Q2 (c13)
    0x66, // Q3 (c21)
    0x68, // c22
    0x6A, // roll (c23)
    0x6C, // pitch (c31)
    0x6E, // yaw (c32)
    0x70, // c33
    0x70 /* special fake read */
};
static const size_t request_len = sizeof(request) / sizeof(request[0]);
static uint16_t rxbuf[request_len];

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
static uint16_t read(uint8_t address){
  uint16_t ret;

  osalDbgCheck(address < 128);
  spiSelect(&ADIS_SPI);
  ret = spiPolledExchange(&ADIS_SPI, address << 8);
  spiUnselect(&ADIS_SPI);
  chSysPolledDelayX(ADIS_NSS_DELAY_US);

  return ret;
}

/**
 *
 */
static void write(uint8_t address, uint16_t word){
  osalDbgCheck(address < 64);

  spiSelect(&ADIS_SPI);
  spiPolledExchange(&ADIS_SPI, (1 << 15) | (address << 8) | (word & 0xFF));
  spiUnselect(&ADIS_SPI);
  chSysPolledDelayX(ADIS_NSS_DELAY_US);

  spiSelect(&ADIS_SPI);
  spiPolledExchange(&ADIS_SPI, (1 << 15) | ((address + 1) << 8) | ((word >> 8) & 0xFF));
  spiUnselect(&ADIS_SPI);
  chSysPolledDelayX(ADIS_NSS_DELAY_US);
}

/**
 * @note    This function does not use defined write() because page select
 *          is special case of write - single byte.
 */
static void select_page(uint8_t page){

  osalDbgCheck(page < 13);
  spiSelect(&ADIS_SPI);
  spiPolledExchange(&ADIS_SPI, (1 << 15) | page);
  spiUnselect(&ADIS_SPI);
  chSysPolledDelayX(ADIS_NSS_DELAY_US);
}

/**
 *
 */
static bool check_id(void){

  uint16_t ack;
  size_t i;

  read(PROD_ID);
  ack = read(PROD_ID);

  for (i=0; i<supported_models_cnt; i++){
    if (supported_models[i] == ack)
      return OSAL_SUCCESS;
  }
  return OSAL_FAILED;
}

/**
 *
 */
static bool selftest(void){

  uint16_t status;

  select_page(3);
  write(GLOB_CMD, 1 << 1);
  osalThreadSleepMilliseconds(12);
  select_page(0);

  status = read(DIAG_STS);
  if (0 == status)
    return OSAL_SUCCESS;
  else
    return OSAL_FAILED;
}

/**
 *
 */
template<typename T>
static inline T u16_conv(T scale, uint16_t msb){
  return scale * (int16_t)msb;
}

/**
 *
 */
template<typename T>
static inline T u32_conv(T scale, uint16_t msb, uint16_t lsb){
  return scale * (int32_t)((msb << 16) | lsb);
}

/**
 *
 */
template<typename T>
static inline void u16_block_conv(T scale, const uint16_t *raw, T *ret, size_t len){
  for (size_t i = 0; i<len; i++)
    ret[i] = u16_conv(scale, raw[i]);
}

/**
 *
 */
template<typename T>
static inline void u32_block_conv(T scale, const uint16_t *raw, T *ret, size_t len){
  for (size_t i = 0; i<len; i++)
    ret[i] = u32_conv(scale, raw[2*i+1], raw[2*i]);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
Adis::Adis(void):
ready(false)
{
  chTMObjectInit(&tm);
  return;
}

/**
 *
 */
bool Adis::start(void){
  adis_reset_clear();
  spiStart(&ADIS_SPI, &spicfg);
  osalThreadSleepMilliseconds(ADIS_START_TIME_MS);

  if (OSAL_SUCCESS != check_id())
    return OSAL_FAILED;

  if (OSAL_SUCCESS != selftest())
    return OSAL_FAILED;

  /* set sample rate */
  select_page(3);
  write(0x0C, ADIS_SAMPLE_RATE_DIV - 1);
  select_page(0);

  Exti.adis(true);
  ready = true;
  return OSAL_SUCCESS;
}

/**
 *
 */
void Adis::stop(void){
  adis_reset_assert();
  spiStop(&ADIS_SPI);
  ready = false;
}

/**
 * @note    If you do not need some data than pass NULL pointer.
 */
uint16_t Adis::get(adisfp *acc, adisfp *gyr, adisfp *mag,
    adisfp *baro, adisfp *quat, adisfp *euler){

  const adisfp gyr_scale   = 0.00000030517578125; /* to deg/s */
  const adisfp acc_scale   = 0.00000001220703125; /* to G */
  const adisfp mag_scale   = 0.0001; /* to millygauss */
  const adisfp baro_scale  = 0.04; /* to millybars */
  const adisfp temp_scale  = 0.00565; /* to celsius */
  const adisfp quat_scale  = 0.000030517578125;
  const adisfp euler_scale = 0.0054931640625; /* to deg (360/65536) */
  uint16_t errors;
  adisfp temp;

  chTMStartMeasurementX(&tm);
  osalDbgCheck(ready);

  /* reading data */
  read(request[0]); /* first read for warm up */
  for (size_t i=1; i<request_len; i++) // NOTE: this loop must be start from #1
    rxbuf[i-1] = read(request[i]);

  /* converting to useful values */
  temp = 25 + u16_conv(temp_scale, rxbuf[1]);

  if (nullptr != acc)
    u32_block_conv(acc_scale, &rxbuf[8], acc, 3);

  if (nullptr != gyr)
    u32_block_conv(gyr_scale, &rxbuf[2], gyr, 3);

  if (nullptr != mag)
    u16_block_conv(mag_scale, &rxbuf[14], mag, 3);

  if (nullptr != baro)
    *baro = u32_conv(baro_scale, rxbuf[14], rxbuf[15]);

  if (nullptr != quat)
    u16_block_conv(quat_scale, &rxbuf[19], quat, 4);

  if (nullptr != euler)
    u16_block_conv(euler_scale, &rxbuf[24], euler, 3);

  errors = rxbuf[0];

  (void)temp;

  chTMStopMeasurementX(&tm);
  return errors;
}

/**
 *
 */
void Adis::extiISR(EXTDriver *extp, expchannel_t channel){
  (void)extp;
  (void)channel;

  osalSysLockFromISR();
  adis_sem.signalI();
  osalSysUnlockFromISR();
}

/**
 *
 */
msg_t Adis::wait(systime_t timeout){
  return adis_sem.waitTimeout(timeout);
}

/**
 *
 */
float Adis::dt(void){
  float ret = ADIS_SAMPLE_RATE_DIV;
  ret /= ADIS_INTERNAL_SAMPLE_RATE;
  return  ret;
}
