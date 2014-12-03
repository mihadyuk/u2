/*
    ChibiOS/RT - Copyright (C) 2014 Uladzimir Pylinsky aka barthess

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <string.h>

#include "onewire.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define ONEWIRE_ZERO_WIDTH            60
#define ONEWIRE_ONE_WIDTH             6
#define ONEWIRE_SAMPLE_WIDTH          15
#define ONEWIRE_RECOVERY_WIDTH        10
#define ONEWIRE_RESET_LOW_WIDTH       480
#define ONEWIRE_RESET_SAMPLE_WIDTH    550
#define ONEWIRE_RESET_TOTAL_WIDTH     960

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

OWDriver OWD1;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */
static void ow_reset_cb(PWMDriver *pwmp, OWDriver *owp);
static void pwm_reset_cb(PWMDriver *pwmp);
static void ow_read_bit_cb(PWMDriver *pwmp, OWDriver *owp);
static void pwm_read_bit_cb(PWMDriver *pwmp);
static void ow_write_bit_cb(PWMDriver *pwmp, OWDriver *owp);
static void pwm_write_bit_cb(PWMDriver *pwmp);
static void ow_search_rom_cb(PWMDriver *pwmp, OWDriver *owp);
static void pwm_search_rom_cb(PWMDriver *pwmp);

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 * config for fast initializing
 */
static const PWMConfig pwm_default_cfg = {
  1000000,
  ONEWIRE_RESET_TOTAL_WIDTH,
  NULL,
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

/*
 * Look up table for fash crc calculation
 */
static const uint8_t onewire_crc_table[256] = {
     0,     94,   188,   226,    97,  63,   221,   131, 194,   156, 126,   32,    163,   253,   31,     65,
    157,   195,    33,   127,   252, 162,    64,   30,   95,    1,  227,  189,    62,     96,  130,    220,
     35,   125,   159,   193,    66,  28,   254,   160, 225,   191,  93,    3,    128,   222,   60,     98,
    190,   224,     2,    92,   223, 129,    99,   61,  124,    34, 192,  158,    29,     67,  161,    255,
     70,    24,   250,   164,    39, 121,   155,   197, 132,   218,  56,  102,    229,   187,   89,     7,
    219,   133,   103,    57,   186, 228,     6,   88,   25,    71, 165,  251,    120,    38,  196,    154,
    101,    59,   217,   135,    4,   90,   184,   230, 167,   249,  27,   69,    198,   152,  122,     36,
    248,   166,    68,    26,   153, 199,    37,   123,  58,   100, 134,  216,    91,     5,   231,    185,
    140,   210,    48,   110,   237, 179,    81,   15,   78,    16, 242,  172,    47,    113,  147,    205,
     17,    79,   173,   243,   112,  46,   204,   146, 211,   141, 111,   49,    178,   236,   14,     80,
    175,   241,    19,    77,   206, 144,   114,   44,  109,    51, 209,  143,    12,     82,  176,    238,
     50,   108,   142,   208,    83,  13,   239,   177, 240,   174,  76,   18,    145,   207,   45,    115,
    202,   148,   118,    40,   171, 245,    23,   73,    8,    86, 180,  234,    105,    55,  213,    139,
     87,     9,   235,   181,    54, 104,   138,   212, 149,   203,  41,  119,    244,   170,   72,     22,
    233,   183,    85,    11,   136, 214,    52,   106,  43,   117, 151,  201,    74,     20,  246,    168,
    116,    42,   200,   150,    21,  75,   169,   247, 182,   232,  10,   84,    215,   137,  107,    53
};

static time_measurement_t search_rom_tm;

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
static void pwm_reset_cb(PWMDriver *pwmp) {
  ow_reset_cb(pwmp, &OWD1);
}

/**
 *
 */
static void pwm_read_bit_cb(PWMDriver *pwmp) {
  ow_read_bit_cb(pwmp, &OWD1);
}

/**
 *
 */
static void pwm_write_bit_cb(PWMDriver *pwmp) {
  ow_write_bit_cb(pwmp, &OWD1);
}

/**
 *
 */
static void pwm_search_rom_cb(PWMDriver *pwmp) {
  ow_search_rom_cb(pwmp, &OWD1);
}

/**
 *
 */
static void ow_write_bit_I(OWDriver *owp, uint8_t bit) {
#if SYNTH_SEARCH_TEST
  _synth_ow_write_bit(owp, bit);
#else
  osalSysLockFromISR();
  if (0 == bit)
    pwmEnableChannelI(owp->config->pwmd, owp->config->master_channel, ONEWIRE_ZERO_WIDTH);
  else
    pwmEnableChannelI(owp->config->pwmd, owp->config->master_channel, ONEWIRE_ONE_WIDTH);
  osalSysUnlockFromISR();
#endif
}

/**
 *
 */
static uint_fast8_t ow_read_bit_X(void) {
#if SYNTH_SEARCH_TEST
  return _synth_ow_read_bit();
#else
  return palReadPad(GPIOB, GPIOB_TACHOMETER);
#endif
}

/*
 * presence pulse callback
 */
static void ow_reset_cb(PWMDriver *pwmp, OWDriver *owp) {

  owp->reg.slave_present = (PAL_LOW == ow_read_bit_X());

  osalSysLockFromISR();
  pwmDisableChannelI(pwmp, owp->config->sample_channel);
  osalThreadResumeI(&owp->thread, MSG_OK);
  osalSysUnlockFromISR();
}

/**
 *
 */
static void ow_read_bit_cb(PWMDriver *pwmp, OWDriver *owp) {

  if (true == owp->reg.final_timeslot) {
    osalSysLockFromISR();
    pwmDisableChannelI(pwmp, owp->config->sample_channel);
    osalThreadResumeI(&owp->thread, MSG_OK);
    osalSysUnlockFromISR();
    return;
  }
  else {
    *owp->buf |= ow_read_bit_X() << owp->reg.bit;
    owp->reg.bit++;
    if (8 == owp->reg.bit) {
      owp->reg.bit = 0;
      owp->buf++;
      owp->reg.bytes--;
      if (0 == owp->reg.bytes) {
        owp->reg.final_timeslot = true;
        osalSysLockFromISR();
        /* Note: sample channel must be stopped later because it
           must generate one more interrupt */
        pwmDisableChannelI(pwmp, owp->config->master_channel);
        osalSysUnlockFromISR();
      }
    }
  }
}

#include "pads.h"
/*
 * bit transmission callback
 */
static void ow_write_bit_cb(PWMDriver *pwmp, OWDriver *owp) {

  if (8 == owp->reg.bit) {
    owp->buf++;
    owp->reg.bit = 0;
    owp->reg.bytes--;

    if (0 == owp->reg.bytes) {
      osalSysLockFromISR();
      pwmDisableChannelI(pwmp, owp->config->master_channel);
      osalSysUnlockFromISR();
      /* special value to signalizes premature stop protector*/
      owp->reg.final_timeslot = true;
      return;
    }
  }

  /* wait until timer generate last pulse */
  if (true == owp->reg.final_timeslot) {
    #if ONEWIRE_USE_PARASITIC_POWER
    if (owp->reg.need_pullup) {
      owp->reg.state = OW_PULL_UP;
      owp->config->onewire_pullup_assert();
      owp->reg.need_pullup = false;
    }
    #endif

    osalSysLockFromISR();
    osalThreadResumeI(&owp->thread, MSG_OK);
    osalSysUnlockFromISR();
    return;
  }

  ow_write_bit_I(owp, (*owp->buf >> owp->reg.bit) & 1);
  owp->reg.bit++;
}

/**
 * @brief   Helper for collision handler
 */
static void store_bit(OWSearchRom *sr, uint_fast8_t bit) {

  size_t rb = sr->reg.rombit;

  /*            /  8                % 8  */
  sr->retbuf[rb >> 3] |= bit << (rb & 7);
  sr->reg.rombit++;
}

/**
 * @brief   Helper for collision handler
 */
static uint_fast8_t extract_path_bit(const uint8_t *path, uint_fast8_t bit) {
  return (path[bit >> 3] >> (bit & 7)) & 1;
}

/**
 *
 */
static uint_fast8_t collision_handler(OWSearchRom *sr) {

  uint_fast8_t bit;

  switch(sr->reg.search_iter) {
  case OW_SEARCH_ROM_NEXT:
    if ((int)sr->reg.rombit < sr->last_zero_branch) {
      bit = extract_path_bit(sr->prev_path, sr->reg.rombit);
      if (0 == bit) {
        sr->prev_zero_branch = sr->reg.rombit;
        sr->reg.result = OW_SEARCH_ROM_SUCCESS;
      }
      store_bit(sr, bit);
      return bit;
    }
    else if ((int)sr->reg.rombit == sr->last_zero_branch) {
      sr->last_zero_branch = sr->prev_zero_branch;
      store_bit(sr, 1);
      return 1;
    }
    else {
      /* found next branch some levels deeper */
      sr->prev_zero_branch = sr->last_zero_branch;
      sr->last_zero_branch = sr->reg.rombit;
      store_bit(sr, 0);
      sr->reg.result = OW_SEARCH_ROM_SUCCESS;
      return 0;
    }
    break;

  case OW_SEARCH_ROM_FIRST:
    /* always take 0-branch */
    sr->prev_zero_branch = sr->last_zero_branch;
    sr->last_zero_branch = sr->reg.rombit;
    store_bit(sr, 0);
    sr->reg.result = OW_SEARCH_ROM_SUCCESS;
    return 0;
    break;

  default:
    osalSysHalt("Unhandled case");
    return 0; /* warning supressor */
    break;
  }
}

/**
 *
 */
static void ow_search_rom_cb(PWMDriver *pwmp, OWDriver *owp) {

  chTMStartMeasurementX(&search_rom_tm);

  OWSearchRom *sr = &owp->search_rom;

  if (0 == sr->reg.bit_step) {                    /* read direct bit */
    sr->reg.bit_buf |= ow_read_bit_X();
    sr->reg.bit_step++;
  }
  else if (1 == sr->reg.bit_step) {               /* read complement bit */
    sr->reg.bit_buf |= ow_read_bit_X() << 1;
    sr->reg.bit_step++;
    switch(sr->reg.bit_buf){
    case 0b11:
      /* no one device on bus */
      sr->reg.result = OW_SEARCH_ROM_ERROR;
      goto THE_END;
      break;
    case 0b01:
      /* all slaves have 1 in this position */
      store_bit(sr, 1);
      ow_write_bit_I(owp, 1);
      break;
    case 0b10:
      /* all slaves have 0 in this position */
      store_bit(sr, 0);
      ow_write_bit_I(owp, 0);
      break;
    case 0b00:
      /* collision */
      sr->reg.single_device = false;
      ow_write_bit_I(owp, collision_handler(sr));
      break;
    }
  }
  else {                                      /* start next step */
#if !SYNTH_SEARCH_TEST
    ow_write_bit_I(owp, 1);
#endif
    sr->reg.bit_step = 0;
    sr->reg.bit_buf = 0;
  }

  /* one ROM successfully discovered */
  if (64 == sr->reg.rombit) {
    sr->reg.devices_found++;
    sr->reg.search_iter = OW_SEARCH_ROM_NEXT;
    if (true == sr->reg.single_device)
      sr->reg.result = OW_SEARCH_ROM_LAST;
    goto THE_END;
  }

  /* next search bit iteration */
  chTMStopMeasurementX(&search_rom_tm);
  return;

THE_END:
#if SYNTH_SEARCH_TEST
  (void)pwmp;
  return;
#else
  chTMStopMeasurementX(&search_rom_tm);
  osalSysLockFromISR();
  pwmDisableChannelI(pwmp, owp->config->master_channel);
  pwmDisableChannelI(pwmp, owp->config->sample_channel);
  osalThreadResumeI(&(owp)->thread, MSG_OK);
  osalSysUnlockFromISR();
#endif
}

/**
 * @brief   Early reset. Call it once before search rom routine.
 */
static void search_clean_start(OWSearchRom *sr) {

  sr->reg.single_device = true; /* presume simplest way at beginning */
  sr->reg.result = OW_SEARCH_ROM_LAST;
  sr->reg.search_iter = OW_SEARCH_ROM_FIRST;
  sr->retbuf = NULL;
  sr->reg.devices_found = 0;
  memset(sr->prev_path, 0, 8);

  sr->reg.rombit = 0;
  sr->reg.bit_step = 0;
  sr->reg.bit_buf = 0;
  sr->last_zero_branch = -1;
  sr->prev_zero_branch = -1;
}

/**
 * @brief   Call it the begining of every iteration
 */
static void search_clean_iteration(OWSearchRom *sr) {

  sr->reg.rombit = 0;
  sr->reg.bit_step = 0;
  sr->reg.bit_buf = 0;
  sr->reg.result = OW_SEARCH_ROM_LAST;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
void onewireObjectInit(OWDriver *owp) {

  owp->config = NULL;
  owp->reg.slave_present = false;
  owp->reg.state = OW_STOP;
  owp->thread = NULL;

  owp->reg.bytes = 0;
  owp->reg.bit = 0;
  owp->reg.final_timeslot = false;
  owp->buf = NULL;

  owp->pwmcfg = pwm_default_cfg;

#if ONEWIRE_USE_PARASITIC_POWER
  owp->reg.need_pullup = false;
#endif
}

/**
 *
 */
void onewireStart(OWDriver *owp, const OWConfig *config) {

  osalDbgCheck((NULL != owp) && (NULL != config));
#if ONEWIRE_USE_PARASITIC_POWER
  osalDbgCheck((NULL != config->onewire_pullup_assert) &&
               (NULL != config->onewire_pullup_assert));
#endif

  chTMObjectInit(&search_rom_tm);
  owp->config = config;
  owp->reg.state = OW_READY;
}

/**
 *
 */
void onewireStop(OWDriver *owp) {

#if ONEWIRE_USE_PARASITIC_POWER
  owp->config->onewire_pullup_release();
#endif
  owp->config = NULL;
  owp->reg.state = OW_STOP;
}

/**
 *
 */
uint8_t onewireCRC(const uint8_t *buf, size_t len) {
  uint8_t ret = 0;
  size_t i;

  for (i=0; i<len; i++)
    ret = onewire_crc_table[ret ^ buf[i]];

  return ret;
}

/**
 * @brief     Return true if device(s) detected on bus.
 */
bool onewireReset(OWDriver *owp) {
  PWMDriver *pwmd = owp->config->pwmd;

  osalDbgAssert(owp->reg.state == OW_READY, "invalid state");

  /* short circuit on bus or any other device transmit data */
  if (0 == ow_read_bit_X())
    return false;

  owp->pwmcfg.period = ONEWIRE_RESET_LOW_WIDTH + ONEWIRE_RESET_SAMPLE_WIDTH;
  owp->pwmcfg.callback = NULL;
  owp->pwmcfg.channels[owp->config->master_channel].callback = NULL;
  owp->pwmcfg.channels[owp->config->master_channel].mode = PWM_OUTPUT_ACTIVE_LOW;
  owp->pwmcfg.channels[owp->config->sample_channel].callback = pwm_reset_cb;
  owp->pwmcfg.channels[owp->config->sample_channel].mode = PWM_OUTPUT_ACTIVE_LOW;

  pwmStart(pwmd, &owp->pwmcfg);
  pwmEnableChannel(pwmd, owp->config->master_channel, ONEWIRE_RESET_LOW_WIDTH);
  pwmEnableChannel(pwmd, owp->config->sample_channel, ONEWIRE_RESET_SAMPLE_WIDTH);
  pwmEnableChannelNotification(pwmd, owp->config->sample_channel);

  osalSysLock();
  osalThreadSuspendS(&owp->thread);
  osalSysUnlock();

  pwmStop(pwmd);

  /* wait until slave release bus to discriminate it from short circuit */
  osalThreadSleepMicroseconds(500);
  return (1 == ow_read_bit_X()) && (true == owp->reg.slave_present);
}

/**
 *
 */
void onewireRead(OWDriver *owp, uint8_t *rxbuf, size_t rxbytes) {
  PWMDriver *pwmd = owp->config->pwmd;

  osalDbgAssert(owp->reg.state == OW_READY, "invalid state");

  /* Buffer zeroing. This is important because of driver collects
     bits using |= operation.*/
  memset(rxbuf, 0, rxbytes);

  owp->reg.bit = 0;
  owp->reg.final_timeslot = false;
  owp->buf = rxbuf;
  owp->reg.bytes = rxbytes;

  owp->pwmcfg.period = ONEWIRE_ZERO_WIDTH + ONEWIRE_RECOVERY_WIDTH;
  owp->pwmcfg.callback = NULL;
  owp->pwmcfg.channels[owp->config->master_channel].callback = NULL;
  owp->pwmcfg.channels[owp->config->master_channel].mode = PWM_OUTPUT_ACTIVE_LOW;
  owp->pwmcfg.channels[owp->config->sample_channel].callback = pwm_read_bit_cb;
  owp->pwmcfg.channels[owp->config->sample_channel].mode = PWM_OUTPUT_ACTIVE_LOW;

  pwmStart(pwmd, &owp->pwmcfg);
  pwmEnableChannel(pwmd, owp->config->master_channel, ONEWIRE_ONE_WIDTH);
  pwmEnableChannel(pwmd, owp->config->sample_channel, ONEWIRE_SAMPLE_WIDTH);
  pwmEnableChannelNotification(pwmd, owp->config->sample_channel);

  osalSysLock();
  osalThreadSuspendS(&owp->thread);
  osalSysUnlock();

  pwmStop(pwmd);
}

/**
 *
 */
void onewireWrite(OWDriver *owp, uint8_t *txbuf,
                size_t txbytes, systime_t pullup_time) {
  PWMDriver *pwmd = owp->config->pwmd;

  osalDbgAssert(owp->reg.state == OW_READY, "invalid state");

#if !ONEWIRE_USE_PARASITIC_POWER
  osalDbgAssert(0 == pullup_time, "Non zero pull up time is valid only in parasitic power mode");
#endif

  owp->buf = txbuf;
  owp->reg.bit = 0;
  owp->reg.final_timeslot = false;
  owp->reg.bytes = txbytes;

  owp->pwmcfg.period = ONEWIRE_ZERO_WIDTH + ONEWIRE_RECOVERY_WIDTH;
  owp->pwmcfg.callback = pwm_write_bit_cb;
  owp->pwmcfg.channels[owp->config->master_channel].callback = NULL;
  owp->pwmcfg.channels[owp->config->master_channel].mode = PWM_OUTPUT_ACTIVE_LOW;
  owp->pwmcfg.channels[owp->config->sample_channel].callback = NULL;
  owp->pwmcfg.channels[owp->config->sample_channel].mode = PWM_OUTPUT_DISABLED;

#if ONEWIRE_USE_PARASITIC_POWER
  if (pullup_time > 0) {
    owp->reg.state = OW_PULL_UP;
    owp->reg.need_pullup = true;
  }
#endif

  pwmStart(pwmd, &owp->pwmcfg);
  pwmEnablePeriodicNotification(pwmd);

  osalSysLock();
  osalThreadSuspendS(&owp->thread);
  osalSysUnlock();

  pwmDisablePeriodicNotification(pwmd);
  pwmStop(pwmd);

#if ONEWIRE_USE_PARASITIC_POWER
  if (pullup_time > 0) {
    osalThreadSleep(pullup_time);
    owp->config->onewire_pullup_release();
    owp->reg.state = OW_READY;
  }
#endif
}

/**
 * @brief   Perform tree search on bus.
 * @note    This function does internal 1-wire reset calls every search
 *          iteration.
 *
 * @param[in] owp         pointer to a @p OWDriver object
 * @param[out] result     pointer to buffer for founded ROMs
 * @param[in] max_rom_cnt buffer size in ROMs count for overflow prevention
 *
 * @return              Count of discovered ROMs. May be more than max_rom_cnt.
 * @retval 0            no ROMs found or communication error occurred.
 */
size_t onewireSearchRom(OWDriver *owp, uint8_t *result, size_t max_rom_cnt) {
  PWMDriver *pwmd = owp->config->pwmd;
  uint8_t cmd = ONEWIRE_CMD_SEARCH_ROM;

  osalDbgAssert(OW_READY == owp->reg.state, "invalid state");

  search_clean_start(&owp->search_rom);

  do {
    /* every search must be started from clean state */
    if (false == onewireReset(owp))
      return 0;

    /* initialize buffer to store result */
    if (owp->search_rom.reg.devices_found >= max_rom_cnt)
      owp->search_rom.retbuf = result + 8*(max_rom_cnt-1);
    else
      owp->search_rom.retbuf = result + 8*owp->search_rom.reg.devices_found;
    memset(owp->search_rom.retbuf, 0, 8);

    /* clean iteration state */
    search_clean_iteration(&owp->search_rom);

    /**/
    onewireWrite(&OWD1, &cmd, 1, 0);

    /* Reconfiguration always needed because of previous call onewireWrite.*/
    owp->pwmcfg.period = ONEWIRE_ZERO_WIDTH + ONEWIRE_RECOVERY_WIDTH;
    owp->pwmcfg.callback = NULL;
    owp->pwmcfg.channels[owp->config->master_channel].callback = NULL;
    owp->pwmcfg.channels[owp->config->master_channel].mode = PWM_OUTPUT_ACTIVE_LOW;
    owp->pwmcfg.channels[owp->config->sample_channel].callback = pwm_search_rom_cb;
    owp->pwmcfg.channels[owp->config->sample_channel].mode = PWM_OUTPUT_ACTIVE_LOW;
    pwmStart(pwmd, &owp->pwmcfg);
    pwmEnableChannel(pwmd, owp->config->master_channel, ONEWIRE_ONE_WIDTH);
    pwmEnableChannel(pwmd, owp->config->sample_channel, ONEWIRE_SAMPLE_WIDTH);
    pwmEnableChannelNotification(pwmd, owp->config->sample_channel);

    osalSysLock();
    osalThreadSuspendS(&owp->thread);
    osalSysUnlock();

    pwmStop(pwmd);

    if (OW_SEARCH_ROM_ERROR != owp->search_rom.reg.result) {
      /* check CRC and return 0 (error status) if mismatch */
      if (owp->search_rom.retbuf[7] != onewireCRC(owp->search_rom.retbuf, 7))
        return 0;
      /* store cached result for usage in next iteration */
      memcpy(owp->search_rom.prev_path, owp->search_rom.retbuf, 8);
    }
  }
  while (OW_SEARCH_ROM_SUCCESS == owp->search_rom.reg.result);

  /**/
  if (OW_SEARCH_ROM_ERROR == owp->search_rom.reg.result)
    return 0;
  else
    return owp->search_rom.reg.devices_found;
}

#if SYNTH_SEARCH_TEST
#include "onewire_sr_synth.c"
#endif /* SYNTH_SEARCH_TEST */
