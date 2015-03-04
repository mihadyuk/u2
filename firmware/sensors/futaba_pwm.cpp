#pragma GCC optimize "-O0"

#include "main.h"
#include "pads.h"
#include "futaba_pwm.hpp"
#include "mavlink_local.hpp"

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
extern mavlink_rc_channels_raw_t    mavlink_out_rc_channels_raw_struct;
extern mavlink_rc_channels_scaled_t mavlink_out_rc_channels_scaled_struct;

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

uint16_t FutabaPWM::cache[FUTABA_PWM_CHANNELS];

/**
 *
 */
void futaba_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p) {
  (void)eicup;
  (void)channel;

  if (EICU_INPUT_PULSE == eicup->channel[channel].config->mode)
    FutabaPWM::cache[channel] = w;
  else {
    /* debug output */
    FutabaPWM::cache[0] = w;
    FutabaPWM::cache[1] = p;
  }
}

/**
 *
 */
static EICUChannelConfig futabacfg = {
    EICU_INPUT_ACTIVE_HIGH,
    EICU_INPUT_PULSE,
    futaba_cb
};

/**
 *
 */
static EICUConfig eicucfg = {
    (1000 * 1000),      /* EICU clock frequency (Hz).*/
    {
        &futabacfg,
        &futabacfg,
        &futabacfg,
        &futabacfg,
    },
    0
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
static void futaba2mavlink(const uint16_t *pwm) {

  mavlink_out_rc_channels_raw_struct.time_boot_ms = TIME_BOOT_MS;
  mavlink_out_rc_channels_raw_struct.chan1_raw = pwm[0];
  mavlink_out_rc_channels_raw_struct.chan2_raw = pwm[1];
  mavlink_out_rc_channels_raw_struct.chan3_raw = pwm[2];
  mavlink_out_rc_channels_raw_struct.chan4_raw = pwm[3];

  mavlink_out_rc_channels_scaled_struct.time_boot_ms = TIME_BOOT_MS;
  mavlink_out_rc_channels_scaled_struct.chan1_scaled = pwm[0];
  mavlink_out_rc_channels_scaled_struct.chan2_scaled = pwm[1];
  mavlink_out_rc_channels_scaled_struct.chan3_scaled = pwm[2];
  mavlink_out_rc_channels_scaled_struct.chan4_scaled = pwm[3];
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void FutabaPWM::start(void) {

  for(size_t i=0; i<FUTABA_PWM_CHANNELS; i++)
    cache[i] = 1500;

  eicuStart(&EICUD4, &eicucfg);
  eicuEnable(&EICUD4);

  ready = true;
}

/**
 *
 */
void FutabaPWM::stop(void) {

  ready = false;
  eicuDisable(&EICUD4);
  eicuStop(&EICUD4);
}

/**
 *
 */
void FutabaPWM::update(uint16_t *pwm) {
  osalDbgCheck(ready);
  memcpy(pwm, cache, sizeof(cache));
  futaba2mavlink(cache);
}
