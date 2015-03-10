#include "main.h"
#include "pads.h"
#include "receiver_pwm.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"
#include <cstring>

using namespace chibios_rt;
using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define CHANNEL_CNT     4

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_rc_channels_t          mavlink_out_rc_channels_struct;

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

static uint16_t cache[CHANNEL_CNT];

/**
 *
 */
void futaba_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p) {
  (void)eicup;
  (void)channel;
  (void)p;

  cache[channel] = w;
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
float ReceiverPWM::get_ch(int32_t map) const {
  if (map != -1)
    return pwm_normalize(cache[map]);
  else
    return 0;
}

/**
 *
 */
void futaba2mavlink(const uint16_t *pwm) {
  // A value of UINT16_MAX implies the channel is unused.

  memset(&mavlink_out_rc_channels_struct, 0xFF, sizeof(mavlink_out_rc_channels_struct));
  mavlink_out_rc_channels_struct.time_boot_ms = TIME_BOOT_MS;
  mavlink_out_rc_channels_struct.chan1_raw = pwm[0];
  mavlink_out_rc_channels_struct.chan2_raw = pwm[1];
  mavlink_out_rc_channels_struct.chan3_raw = pwm[2];
  mavlink_out_rc_channels_struct.chan4_raw = pwm[3];
  mavlink_out_rc_channels_struct.chancount = 4;
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

  param_registry.valueSearch("RECV_map_ail", &map_ail);
  param_registry.valueSearch("RECV_map_ele", &map_ele);
  param_registry.valueSearch("RECV_map_rud", &map_rud);
  param_registry.valueSearch("RECV_map_thr", &map_thr);
  param_registry.valueSearch("RECV_map_man", &map_man);

  osalDbgCheck(CHANNEL_CNT > *map_ail);
  osalDbgCheck(CHANNEL_CNT > *map_ele);
  osalDbgCheck(CHANNEL_CNT > *map_rud);
  osalDbgCheck(CHANNEL_CNT > *map_thr);
  osalDbgCheck(CHANNEL_CNT > *map_man);

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
void ReceiverPWM::update(receiver_data_t &result) {

  osalDbgCheck(ready);

  result.ail = get_ch(*map_ail);
  result.ele = get_ch(*map_ele);
  result.rud = get_ch(*map_rud);
  result.thr = get_ch(*map_thr);

  result.man = static_cast<ManualSwitch>(manual_switch.update(cache[*map_man]));

  result.status = 0;
}



