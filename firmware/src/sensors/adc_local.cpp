#include <cmath>

#include "main.h"
#include "global_flags.h"
#include "alpha_beta.hpp"

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
#define ADC_NUM_CHANNELS          6
#define ADC_BUF_DEPTH             64


/* human readable names */
#define ADC_PRESS_DIFF_CH         ADC_CHANNEL_IN10
#define ADC_MPXV_TEMP_CH          ADC_CHANNEL_IN11
#define ADC_CURRENT_SENS_CH       ADC_CHANNEL_IN12
#define ADC_MAIN_SUPPLY_CH        ADC_CHANNEL_IN13
#define ADC_6V_SUPPLY_CH          ADC_CHANNEL_IN14
#define ADC_RESERVED_CH           ADC_CHANNEL_IN15

#define CHANNEL_OFFSET            10

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
  adc_cb,
  adc_eeror_cb,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_480)    |
    ADC_SMPR1_SMP_AN11(ADC_SAMPLE_480)  |
    ADC_SMPR1_SMP_AN12(ADC_SAMPLE_480)  |
    ADC_SMPR1_SMP_AN13(ADC_SAMPLE_480)  |
    ADC_SMPR1_SMP_AN14(ADC_SAMPLE_480)  |
    ADC_SMPR1_SMP_AN15(ADC_SAMPLE_480),
  0,                        /* SMPR2 */
  ADC_SQR1_NUM_CH(ADC_NUM_CHANNELS),
  0,
  ADC_SQR3_SQ6_N(ADC_RESERVED_CH)       |
    ADC_SQR3_SQ5_N(ADC_6V_SUPPLY_CH)    |
    ADC_SQR3_SQ4_N(ADC_MAIN_SUPPLY_CH)  |
    ADC_SQR3_SQ3_N(ADC_CURRENT_SENS_CH) |
    ADC_SQR3_SQ2_N(ADC_MPXV_TEMP_CH)    |
    ADC_SQR3_SQ1_N(ADC_PRESS_DIFF_CH)
};

static filters::AlphaBeta<int32_t, 128> temp_filter;
static filters::AlphaBeta<int32_t, 128> mpxv_filter;

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

  //temp_filter(samples[ADC_MPXV_TEMP_CH - CHANNEL_OFFSET]);
}

/*
 *
 */
static void adc_eeror_cb(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;

  errors++;
}

/**
 *
 */
static adcsample_t do_filter(size_t offset, filters::AlphaBeta<int32_t, 128> &filter) {
  size_t idx;

  for (size_t i=0; i<ADC_BUF_DEPTH; i++) {
    idx = i * ADC_NUM_CHANNELS + offset;
    filter(samples[idx]);
  }

  return filter.get();
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
void ADCInitLocal(void) {

  adcStart(&ADCD1, &adccfg);
  adcStartConversion(&ADCD1, &adccg, samples, ADC_BUF_DEPTH);
}

/**
 *
 */
adcsample_t ADCget6v(void) {
  return samples[ADC_6V_SUPPLY_CH - CHANNEL_OFFSET];
}

/**
 *
 */
adcsample_t ADCgetCurrent(void) {
  return samples[ADC_CURRENT_SENS_CH - CHANNEL_OFFSET];
}

/**
 *
 */
adcsample_t ADCgetMPXVtemp(void) {
  return do_filter(ADC_MPXV_TEMP_CH - CHANNEL_OFFSET, temp_filter);
}

/**
 *
 */
adcsample_t ADCgetMPXV(void) {
  return do_filter(ADC_PRESS_DIFF_CH - CHANNEL_OFFSET, mpxv_filter);
}

