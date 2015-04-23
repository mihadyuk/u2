#include "main.h"
#include "pads.h"
#include "receiver_pwm.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"

using namespace chibios_rt;
using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define CHANNEL_CNT         4

/* middle point in uS */
#define NORMALIZE_SHIFT     1500
/* max - mid. Good futabas have max at 2000uS but not all futabas good enough */
#define NORMALIZE_SCALE     400

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_rc_channels_t          mavlink_out_rc_channels_struct;
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

static uint16_t cache[CHANNEL_CNT];
static systime_t timestamp[CHANNEL_CNT]; // timestamp array for timeout detect

/**
 *
 */
void futaba_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p) {
  (void)eicup;
  (void)channel;
  (void)p;

  cache[channel] = w;
  timestamp[channel] = chVTGetSystemTimeX();
}

/**
 *
 */
static const EICUChannelConfig futabacfg = {
    EICU_INPUT_ACTIVE_HIGH,
    EICU_INPUT_PULSE,
    futaba_cb
};

/**
 *
 */
static const EICUConfig eicucfg = {
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
static void receiver2mavlink(const uint16_t *pwm) {
  // A value of UINT16_MAX implies the channel is unused.

  memset(&mavlink_out_rc_channels_struct, 0xFF, sizeof(mavlink_out_rc_channels_struct));
  mavlink_out_rc_channels_struct.time_boot_ms = TIME_BOOT_MS;
  mavlink_out_rc_channels_struct.chan1_raw = pwm[0];
  mavlink_out_rc_channels_struct.chan2_raw = pwm[1];
  mavlink_out_rc_channels_struct.chan3_raw = pwm[2];
  mavlink_out_rc_channels_struct.chan4_raw = pwm[3];
  mavlink_out_rc_channels_struct.chancount = 4;

  // A value of UINT16_MAX implies the channel is unused.
  mavlink_out_rc_channels_scaled_struct.time_boot_ms = TIME_BOOT_MS;
  mavlink_out_rc_channels_scaled_struct.chan1_scaled = pwm[0];
  mavlink_out_rc_channels_scaled_struct.chan2_scaled = pwm[1];
  mavlink_out_rc_channels_scaled_struct.chan3_scaled = pwm[2];
  mavlink_out_rc_channels_scaled_struct.chan4_scaled = pwm[3];
  mavlink_out_rc_channels_scaled_struct.chan5_scaled = UINT16_MAX;
  mavlink_out_rc_channels_scaled_struct.chan6_scaled = UINT16_MAX;
  mavlink_out_rc_channels_scaled_struct.chan7_scaled = UINT16_MAX;
  mavlink_out_rc_channels_scaled_struct.chan8_scaled = UINT16_MAX;
  mavlink_out_rc_channels_scaled_struct.rssi = 255;
  mavlink_out_rc_channels_scaled_struct.port = 0;
}

/**
 * retval   true == timeout
 *          fasle == OK
 */
static bool check_timeout(int32_t map, systime_t timeout) {
  bool ret;

  osalSysLock();
  if ((chVTGetSystemTimeX() - timestamp[map]) >= timeout) {
    ret = true;
    /* prevent false positive when system timer overflows */
    timestamp[map] = chVTGetSystemTimeX() - timeout;
  }
  else {
    ret = false;
  }
  osalSysUnlock();

  return ret;
}

/**
 *
 */
void ReceiverPWM::get_ch(size_t chnum, float *result, uint32_t *status) const {

  *result = pwm_normalize(cache[chnum], NORMALIZE_SHIFT, NORMALIZE_SCALE);

  if (check_timeout(chnum, MS2ST(this->timeout)))
    *status |= (1 << chnum);
  else
    *status &= ~(1 << chnum);
}

/**
 *
 */
void ReceiverPWM::get_tumbler(size_t chnum, ManualSwitch *result, uint32_t *status) {

  *result = static_cast<ManualSwitch>(manual_switch.update(cache[chnum]));

  if (check_timeout(chnum, MS2ST(this->timeout)))
    *status |= 1 << chnum;
  else
    *status &= ~(1 << chnum);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void ReceiverPWM::start(const uint32_t *timeout) {

  this->timeout = timeout;

  param_registry.valueSearch("RC_map_man", &map_man);

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
void ReceiverPWM::update(RecevierOutput &result) {

  osalDbgCheck(ready);

  receiver2mavlink(cache);

  for (size_t i=0; i<CHANNEL_CNT; i++) {
    get_ch(i, &result.ch[i], &result.status);
  }

  /* manual switch will be processed separately because I still have no
   * ideas how to do this elegantly inside ACS. */
  if (-1 == *map_man)
    result.man = ManualSwitch::fullauto;
  else
    get_tumbler(*map_man, &result.man, &result.status);
}

