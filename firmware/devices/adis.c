#include "main.h"
#include "pads.h"

#include "adis.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define ADIS_START_TIME_MS        560
#define ADIS_BUF_LEN              8
#define ADIS_SPI                  SPID1

/* address definitions */
#define DIAG_STS      0x0A
#define GLOB_CMD      0x02
#define PROD_ID       0x7E
#define SYS_E_FLAG    0x08

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
 * Software delay for this stupid sensor - 1.5uS
 */
static const size_t ADIS_NSS_DELAY_US = (STM32_SYSCLK + STM32_SYSCLK / 2) / 1000000;

/**
 *
 */
static const uint16_t ADIS_SAMPLE_RATE_DIV = 24;

/**
 *
 */
static const uint16_t ADIS_INTERNAL_SAMPLE_RATE = 2460;

/*
 * currently supported ADIS models
 */
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

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
bool AdisStart(void){
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

  return OSAL_SUCCESS;
}

/**
 *
 */
float AdisDT(void){
  float ret = ADIS_SAMPLE_RATE_DIV;
  ret /= ADIS_INTERNAL_SAMPLE_RATE;
  return  ret;
}

/**
 *
 */
void AdisStop(void){
  adis_reset_assert();
  spiStop(&ADIS_SPI);
}








static const uint8_t request[] = {
    0x0E,
    0x10,
    0x12,
    0x14,
    0x16,
    0x18,
    0x1A,
    0x1C,
    0x1E,
    0x20,
    0x22,
    0x24,
    0x26,
    0x28,
    0x2A,
    0x2C,
    0x2E,
    0x30,
    0x30 /* special fake read */
};
static const size_t request_len = sizeof(request) / sizeof(request[0]);

/**
 * @note    If you do not need some data than pass NULL pointer.
 */
uint16_t AdisRead(float *acc, float *gyr, float *mag, float *baro, float *quat, float *euler){
  uint16_t ack[request_len];
  uint16_t errors = read(SYS_E_FLAG);

  /* start reading data */
  for (size_t i=0; i<request_len; i++)
    ack[i] = read(request[i]);

//  spiSelect(&ADIS_SPI);
//  spiExchange(&ADIS_SPI, ADIS_BUF_LEN, txbuf, rxbuf);
//  spiUnselect(&ADIS_SPI);

  return errors;
}
