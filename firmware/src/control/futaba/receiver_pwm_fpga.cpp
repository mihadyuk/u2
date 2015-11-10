#include "main.h"
#include "fpga_icu.h"

#include "receiver_pwm_fpga.hpp"
#include "param_registry.hpp"
#include "mavlink_local.hpp"

using namespace chibios_rt;
using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define CHANNEL_CNT           4

/* mask with for connected/used channels */
#define INPUT_CHANNEL_MASK    0b1011

/* middle point in uS */
#define NORMALIZE_SHIFT       1500

/* == (max - mid). Good futabas have max at 2000uS but not all futabas good enough */
#define NORMALIZE_SCALE       400

#define MAX_VALID_VALUE       2200
#define MIN_VALID_VALUE       800

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
static void receiver2mavlink(const uint16_t *pwm, size_t channels) {

  // A value of UINT16_MAX implies the channel is unused.
  memset(&mavlink_out_rc_channels_struct, 0xFF, sizeof(mavlink_out_rc_channels_struct));
  mavlink_out_rc_channels_struct.time_boot_ms = TIME_BOOT_MS;
  mavlink_out_rc_channels_struct.chan1_raw = pwm[0];
  mavlink_out_rc_channels_struct.chan2_raw = pwm[1];
  mavlink_out_rc_channels_struct.chan3_raw = pwm[2];
  mavlink_out_rc_channels_struct.chan4_raw = pwm[3];
  mavlink_out_rc_channels_struct.chancount = channels;
  mavlink_out_rc_channels_struct.rssi = 255;

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
 *
 */
uint16_t ReceiverPWMFPGA::get_ch(size_t chnum, bool *data_valid) const {

  uint16_t ret = fpgaicuRead(&FPGAICUD1, chnum);

  if ((ret > MAX_VALID_VALUE) || (ret < MIN_VALID_VALUE))
    *data_valid = false;

  return ret;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void ReceiverPWMFPGA::start(void) {

  fpgaicuStart(&FPGAICUD1, &FPGAD1);

  ready = true;
}

/**
 *
 */
void ReceiverPWMFPGA::stop(void) {

  ready = false;

  eicuDisable(&EICUD4);
  eicuStop(&EICUD4);
}

/**
 *
 */
void ReceiverPWMFPGA::update(RecevierOutput &result) {
  bool data_valid = true;

  osalDbgCheck(ready);

  receiver2mavlink(FPGAICUD1.icu, CHANNEL_CNT);

  /* fill all with valid values */
  for (size_t i=0; i<ArrayLen(result.ch); i++)
    result.ch[i] = 1500;

  /* overwrite actual values with acquired results */
  for (size_t i=0; i<CHANNEL_CNT; i++) {
    if (1 == ((INPUT_CHANNEL_MASK >> i) & 1))
      result.ch[i] = get_ch(i, &data_valid);
    else
      result.ch[i] = NORMALIZE_SHIFT; /* special case for unconnected input */
  }

  result.data_valid = data_valid;
  result.normalize_scale = NORMALIZE_SCALE;
  result.normalize_shift = NORMALIZE_SHIFT;
  result.channels = CHANNEL_CNT;
}

