#include "main.h"
#include "pads.h"
#include "receiver_pwm.hpp"
#include "mavlink_local.hpp"

using namespace chibios_rt;
using namespace control;

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
extern mavlink_rc_channels_raw_t      mavlink_out_rc_channels_raw_struct;
extern mavlink_rc_channels_scaled_t   mavlink_out_rc_channels_scaled_struct;

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
static receiver_data_t cache;

/**
 *
 */
void futaba_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p) {
  (void)eicup;
  (void)channel;
  (void)p;

  cache.pwm[channel] = w;
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
void ReceiverPWM::start(systime_t timeout) {

  this->timeout = timeout;

  eicuStart(&EICUD4, &eicucfg);
  eicuEnable(&EICUD4);

  ready = true;
}

/**
 *
 */
void ReceiverPWM::stop(void) {

  ready = false;
  eicuDisable(&EICUD4);
  eicuStop(&EICUD4);
}

/**
 *
 */
void ReceiverPWM::update(receiver_data_t &result) const {

  osalDbgCheck(ready);

  osalSysLock();
  result = cache;
  osalSysUnlock();
}



