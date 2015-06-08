#include "main.h"
#include "mavlink_local.hpp"
#include "adc_local.hpp"
#include "mpxv.hpp"
#include "param_registry.hpp"
#include "alpha_beta.hpp"
#include "time_keeper.hpp"
#include "mav_logger.hpp"

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
extern MavLogger mav_logger;

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
static filters::AlphaBeta<float, 64> temp_filter;
volatile size_t spi_delay;
static mavMail dbg_mail;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

static float mpxv_temp(uint16_t raw) {
  int32_t uV = raw * 807;
  const int32_t zero_uV = 1375000;
  //return temp_filter((uV - zero_uV) / 22500.0);
  return (uV - zero_uV) / 22500.0;
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


#if 0
/** термокомпенсация нуля
 * Принимает сырое значение с датчика и температуру в градусах цельсия*/
static uint16_t zerocomp(uint16_t raw, int16_t t){

  putinrange(t, MIN_TEMP, MAX_TEMP);

  uint16_t zero = zerocomp_table[t - MIN_TEMP];

  if (zero >= raw)
    return 0;
  else
    return raw - zero;
}

/* принимает сырое значение с датчика
 * возвращает воздушную скорость в м/с
 *
 * при комнатной температуре смещение нуля 0.201 V, вычитаемое значение 0.183 V,
 * на выходе усилителя 0.167 V
 * КУ получается - 9.277777
 */
#define KU    928     //КУ*100
#define Radc  122070  //uV*100 (чувствительность АЦП 5.0/4096 вольт на деление)
#define Smpx  450     //(Чувствительность датчика 450uV/Pa)

float air_speed(uint16_t press_diff_raw){
  uint16_t p;
  p = zerocomp(press_diff_raw, comp_data.temp_onboard / 10);
  p = ((p * Radc) / Smpx) / KU; /* давление в паскалях */
  return sqrtf((float)(2*p) / 1.2f);
}
#endif



static const uint32_t KU   = 1000;    // КУ*100
static const uint32_t Radc = 80566;   // uV*100 (чувствительность АЦП 3.3/4096 вольт на деление)
static const uint32_t S    = 450;     // Чувствительность датчика (450uV/Pa)

static float air_speed(uint16_t raw) {
  uint32_t p;
  p = ((raw * Radc) / S) / KU; /* давление в паскалях */

  return sqrtf((float)(2*p) / 1.225f);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void MPXV::start(void) {
  param_registry.valueSearch("ADC_mpxv_shift", &mpxv_shift);
  ready = true;
}

/*
 *
 */
float MPXV::get(void) {

  (void)air_speed;
  (void)mpxv_temp;

  softspi_write(*mpxv_shift & 0xFF);

  return mpxv_temp(ADCgetMPXVtemp());
}


void MPXV::__soft_spi_test(void) {
  uint8_t word = 0;

  while(true) {
    softspi_write(word);
    word++;
    osalThreadSleepMilliseconds(10);
  }
}
