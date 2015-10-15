#pragma GCC optimize "-O2"

#include "main.h"
#include "alpha_beta.hpp"

#include "adc_local.hpp"

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define ADC_NUM_CHANNELS          16
#define ADC_BUF_DEPTH             16

#if 0
// channel numbering cheatsheet for bezvodiatel board
#define ADC_PRESS_DIFF_CH         ADC_CHANNEL_IN10
#define ADC_MPXV_TEMP_CH          ADC_CHANNEL_IN11
#define ADC_CURRENT_SENS_CH       ADC_CHANNEL_IN12
#define ADC_MAIN_SUPPLY_CH        ADC_CHANNEL_IN13
#define ADC_6V_SUPPLY_CH          ADC_CHANNEL_IN14
#define ADC_RESERVED_CH           ADC_CHANNEL_IN15
#endif

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */
static void adc_cb(ADCDriver *adcp, adcsample_t *samples, size_t n);
static void adc_eeror_cb(ADCDriver *adcp, adcerror_t err);

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static size_t errors = 0;

static ADCConfig adccfg; // dummy for STM32

static adcsample_t samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

/*
 * ADC conversion group.
 */
static const ADCConversionGroup adccg = {
  TRUE,
  ADC_NUM_CHANNELS,
  NULL,//adc_cb,
  adc_eeror_cb,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_480)    |
    ADC_SMPR1_SMP_AN11(ADC_SAMPLE_480)  |
    ADC_SMPR1_SMP_AN12(ADC_SAMPLE_480)  |
    ADC_SMPR1_SMP_AN13(ADC_SAMPLE_480)  |
    ADC_SMPR1_SMP_AN14(ADC_SAMPLE_480)  |
    ADC_SMPR1_SMP_AN15(ADC_SAMPLE_480),
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_480)     |
    ADC_SMPR2_SMP_AN1(ADC_SAMPLE_480)   |
    ADC_SMPR2_SMP_AN2(ADC_SAMPLE_480)   |
    ADC_SMPR2_SMP_AN3(ADC_SAMPLE_480)   |
    ADC_SMPR2_SMP_AN4(ADC_SAMPLE_480)   |
    ADC_SMPR2_SMP_AN5(ADC_SAMPLE_480)   |
    ADC_SMPR2_SMP_AN5(ADC_SAMPLE_480)   |
    ADC_SMPR2_SMP_AN6(ADC_SAMPLE_480)   |
    ADC_SMPR2_SMP_AN7(ADC_SAMPLE_480)   |
    ADC_SMPR2_SMP_AN8(ADC_SAMPLE_480)   |
    ADC_SMPR2_SMP_AN9(ADC_SAMPLE_480),
  ADC_SQR1_NUM_CH(ADC_NUM_CHANNELS)     |
    ADC_SQR1_SQ16_N(ADC_CHANNEL_IN15)   |
    ADC_SQR1_SQ15_N(ADC_CHANNEL_IN14)   |
    ADC_SQR1_SQ14_N(ADC_CHANNEL_IN13)   |
    ADC_SQR1_SQ13_N(ADC_CHANNEL_IN12),
  ADC_SQR2_SQ12_N(ADC_CHANNEL_IN11) |
    ADC_SQR2_SQ11_N(ADC_CHANNEL_IN10)   |
    ADC_SQR2_SQ10_N(ADC_CHANNEL_IN9)    |
    ADC_SQR2_SQ9_N(ADC_CHANNEL_IN8)     |
    ADC_SQR2_SQ8_N(ADC_CHANNEL_IN7)     |
    ADC_SQR2_SQ7_N(ADC_CHANNEL_IN6),
  ADC_SQR3_SQ6_N(ADC_CHANNEL_IN5)       |
    ADC_SQR3_SQ5_N(ADC_CHANNEL_IN4)     |
    ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3)     |
    ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2)     |
    ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)     |
    ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)
};

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
/*
 * ADC streaming callback.
 */
static void adc_cb(ADCDriver *adcp, adcsample_t *samples, size_t n) {
  (void)adcp;
  (void)samples;
  (void)n;
}

/*
 *
 */
static void adc_eeror_cb(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;

  errors++;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
void ADCLocal::start(void) {
  (void)adc_cb;

  adcStart(&ADCD1, &adccfg);
  adcStartConversion(&ADCD1, &adccg, samples, ADC_BUF_DEPTH);

  ready = true;
}

/**
 *
 */
void ADCLocal::stop(void) {
  (void)adc_cb;

  adcStopConversion(&ADCD1);
  adcStop(&ADCD1);

  ready = true;
}

/**
 *
 */
adcsample_t ADCLocal::getChannel(size_t N, filters::AlphaBetaBase<int32_t> &filter) {
  size_t idx;

  osalDbgCheck(ready);

  for (size_t i=0; i<ADC_BUF_DEPTH; i++) {
    idx = N + i * ADC_BUF_DEPTH;
    filter(samples[idx]);
  }

  return filter.get();
}

/**
 *
 */
adcsample_t ADCLocal::getChannel(size_t N) {
  return samples[N];
}
