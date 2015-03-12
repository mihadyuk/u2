#include "main.h"

#include "adc_local.hpp"
#include "alpha_beta.hpp"
#include "mpxv.hpp"

using namespace chibios_rt;

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
static filters::AlphaBetaFixedLen<float, 64> temp_filter;
volatile size_t spi_delay;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

static float mpxv_temp(uint16_t raw) {
  int32_t uV = raw * 803;
  const int32_t zero_uV = 1375000;
  return temp_filter.update((uV - zero_uV) / 22.500);
}

static inline void cs_low(void)     {palClearPad(GPIOD, GPIOD_AD_CS);}
static inline void cs_high(void)    {palSetPad(GPIOD,   GPIOD_AD_CS);}
static inline void clk_low(void)    {palClearPad(GPIOA, GPIOA_AD_CLK);}
static inline void clk_high(void)   {palSetPad(GPIOA,   GPIOA_AD_CLK);}
static inline void sdi_1(void)      {palSetPad(GPIOD,   GPIOD_AD_SDI);}
static inline void sdi_0(void)      {palClearPad(GPIOD, GPIOD_AD_SDI);}

static inline void clk_delay(size_t ticks) {
  spi_delay = ticks;
  while(spi_delay > 0)
    spi_delay--;
}

static inline void sdi_bit(uint32_t bit) {
  if (0 == bit)
    sdi_0();
  else
    sdi_1();
}

/**
 *
 */
void softspi_write(uint8_t data) {
  size_t i;
  clk_low();
  cs_low();

  for (i=8; i>0; i--) {
    sdi_bit((data >> (i-1)) & 1);
    clk_delay(1);

    clk_high();
    clk_delay(1);
    clk_low();
  }

  /* release bus and wait until IC process data */
  cs_high();
  clk_delay(4);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/*
 *
 */
float MPXV::get(void) {
  return mpxv_temp(ADCgetMPXVtemp());
}


void MPXV::soft_spi_test(void) {
  uint8_t word = 0;

  while(true) {
    softspi_write(word);
    word++;
    osalThreadSleepMilliseconds(10);
  }
}
