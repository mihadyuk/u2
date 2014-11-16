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

#include "main.h"
#include "onewire.h"
#include "pads.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define ZERO_WIDTH        60
#define ONE_WIDTH         6
#define SAMPLE_WIDTH      15
#define RECOVERY_WIDTH    10

#define MASTER_CHANNEL    2 /* this PWM channel drives bus */
#define SAMPLE_CHANNEL    3 /* this just generates interrupts when read needed */

#define READ_ROM              0x33
#define SEARCH_ROM            0xF0
#define MATCH_ROM             0x55
#define CONVERT_TEMPERATURE   0x44
#define READ_SCRATCHPAD       0xBE

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

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static void ow_presece_cb(OWDriver *owp) {
  owp->slave_present = (PAL_LOW == palReadPad(GPIOB, GPIOB_TACHOMETER));
  osalSysLockFromISR();
  osalThreadResumeI(&(owp)->thread, MSG_OK);
  osalSysUnlockFromISR();
}

static void pwm_presence_cb(PWMDriver *pwmp) {
  (void)pwmp;

  ow_presece_cb(&OWD1);
}

/*
 * config for presence detection
 */
static const PWMConfig pwmcfg_presence = {
  1000000,
  960,
  NULL,
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, pwm_presence_cb}
  },
  0,
  0
};

/*
 * bit transmission callback
 */
static void ow_tx_cb(PWMDriver *pwmp, OWDriver *owp) {

  osalSysLockFromISR();

  if (8 == owp->txbit) {
    owp->txbuf++;
    owp->txbit = 0;
    owp->txbytes--;

    if (0 == owp->txbytes) {
      pwmDisableChannelI(pwmp, MASTER_CHANNEL);
      /* special value to signalize premature stop protector
         FIXME: use special driver state for it */
      owp->txbit = 9;
      osalSysUnlockFromISR();
      return;
    }
  }

  /* prevent premature timer stop */
  if (9 == owp->txbit) {
    pwm_lld_disable_periodic_notification(pwmp);
    osalThreadResumeI(&(owp)->thread, MSG_OK);
    osalSysUnlockFromISR();
    return;
  }

  if ((*owp->txbuf & (1 << owp->txbit)) > 0)
    pwmEnableChannelI(pwmp, MASTER_CHANNEL, ONE_WIDTH);
  else
    pwmEnableChannelI(pwmp, MASTER_CHANNEL, ZERO_WIDTH);

  owp->txbit++;

  osalSysUnlockFromISR();
}

static void pwmpcb_tx(PWMDriver *pwmp) {

  ow_tx_cb(pwmp, &OWD1);
}

/*
 * config for bytes transmitting
 */
static const PWMConfig pwmcfg_tx = {
  1000000,                                   /* PWM clock frequency.   */
  (ZERO_WIDTH + RECOVERY_WIDTH),             /* Initial PWM period        */
  pwmpcb_tx,
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

/**
 *
 */
static void pwm_start_rx_cb(PWMDriver *pwmp) {

  osalSysLockFromISR();

  pwmEnableChannelI(pwmp, MASTER_CHANNEL, ONE_WIDTH);
  pwmEnableChannelI(pwmp, SAMPLE_CHANNEL, SAMPLE_WIDTH);
  pwm_lld_disable_periodic_notification(pwmp);
  pwm_lld_enable_channel_notification(pwmp, SAMPLE_CHANNEL);

  osalSysUnlockFromISR();
}

/**
 *
 */
static void ow_read_bit_cb(PWMDriver *pwmp, OWDriver *owp) {

  osalSysLockFromISR();

  *owp->rxbuf |= palReadPad(GPIOB, GPIOB_TACHOMETER) << owp->rxbit;
  owp->rxbit++;
  if (8 == owp->rxbit) {
    owp->rxbit = 0;
    owp->rxbuf++;
    owp->rxbytes--;
    if (0 == owp->rxbytes) {
      pwmDisableChannelI(pwmp, MASTER_CHANNEL);
      pwmDisableChannelI(pwmp, SAMPLE_CHANNEL);
      pwm_lld_disable_channel_notification(pwmp, SAMPLE_CHANNEL);
      osalThreadResumeI(&(owp)->thread, MSG_OK);
    }
  }

  osalSysUnlockFromISR();
}

/**
 *
 */
static void pwm_read_bit_cb(PWMDriver *pwmp) {

  ow_read_bit_cb(pwmp, &OWD1);
}

/*
 * config for data receiving
 */
static const PWMConfig pwmcfg_rx = {
  1000000,                                    /* PWM clock frequency.   */
  (ZERO_WIDTH + RECOVERY_WIDTH),              /* Initial PWM period        */
  pwm_start_rx_cb,
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, pwm_read_bit_cb}
  },
  0,
  0
};






static void ow_search_rom_cb(PWMDriver *pwmp, OWDriver *owp) {

  osalSysLockFromISR();

  if (0 == owp->searchbit) {    /* read bit */
    owp->searchbuf |= palReadPad(GPIOB, GPIOB_TACHOMETER);
    owp->searchbit++;
  }
  else if (1 == owp->searchbit) {    /* read complemented bit */
    owp->searchbuf |= palReadPad(GPIOB, GPIOB_TACHOMETER) << 1;
    owp->searchbit++;

    if (owp->searchbuf == 0b11) {       /* no one device on bus */
      owp->rom = 0;
      osalSysHalt("Temporal debugging trap");
      goto FINISH;
    }
    else if (owp->searchbuf == 0b01) {       /* all slaves have 1 in this position */
      owp->rom |= (uint64_t)1 << owp->rombit;
      pwmEnableChannelI(pwmp, MASTER_CHANNEL, ONE_WIDTH);
    }
    else if (owp->searchbuf == 0b10) { /* all slaves have 0 in this position */
      pwmEnableChannelI(pwmp, MASTER_CHANNEL, ZERO_WIDTH);
    }
    else {      /* collision */
      osalSysHalt("Collision handling unrealized jet");
    }
    owp->rombit++;
  }
  else {
    pwmEnableChannelI(pwmp, MASTER_CHANNEL, ONE_WIDTH);
    owp->searchbit = 0;
  }

  /* one of ROM successfully discovered */
  if (64 == owp->rombit)
    goto FINISH;

  /* next iteration */
  osalSysUnlockFromISR();
  return;

FINISH:
  pwmDisableChannelI(pwmp, MASTER_CHANNEL);
  pwmDisableChannelI(pwmp, SAMPLE_CHANNEL);
  pwm_lld_disable_channel_notification(pwmp, SAMPLE_CHANNEL);
  osalThreadResumeI(&(owp)->thread, MSG_OK);

  osalSysUnlockFromISR();
}

/**
 *
 */
static void pwm_search_rom_cb(PWMDriver *pwmp) {

  ow_search_rom_cb(pwmp, &OWD1);
}


/*
 * config for search rom
 */
static const PWMConfig pwmcfg_search_rom = {
  1000000,                                    /* PWM clock frequency.   */
  (ZERO_WIDTH + RECOVERY_WIDTH),              /* Initial PWM period        */
  NULL,
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, pwm_search_rom_cb}
  },
  0,
  0
};

/*
 *
 */
static const OWConfig ow_cfg = {
  &PWMD4
};

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
uint64_t onewireSearchRom(OWDriver *owp) {
  owp->rom = 0;
  owp->rombit = 0;
  owp->searchbit = 0;
  owp->searchbuf = 0;
  owp->last_collision = 0;

  pwmStart(&PWMD4, &pwmcfg_search_rom);
  pwmEnableChannel(&PWMD4, MASTER_CHANNEL, ONE_WIDTH);
  pwmEnableChannel(&PWMD4, SAMPLE_CHANNEL, SAMPLE_WIDTH);
  pwmEnableChannelNotification(&PWMD4, SAMPLE_CHANNEL);

  osalSysLock();
  osalThreadSuspendS(&owp->thread);
  osalSysUnlock();

  pwmStop(&PWMD4);
  return owp->rom;
}

/**
 *
 */
void onewireObjectInit(OWDriver *owp) {
  owp->config = NULL;
  owp->slave_present = false;
  owp->state = OW_STOP;
  owp->thread = NULL;

  owp->txbytes = 0;
  owp->txbit = 0;
  owp->txbuf = NULL;

  owp->rxbytes = 0;
  owp->rxbit = 0;
  owp->rxbuf = NULL;

  owp->rom = 0;
  owp->rombit = 0;
  owp->searchbit = 0;
  owp->searchbuf = 0;
  owp->last_collision = 0;
}

void onewireStart(OWDriver *owp, const OWConfig *config) {
  owp->config = config;
  owp->state = OW_READY;
}

/**
 *
 */
bool onewireReset(OWDriver *owp) {

  pwmStart(&PWMD4, &pwmcfg_presence);
  pwmEnableChannel(&PWMD4, MASTER_CHANNEL, 480);
  pwmEnableChannel(&PWMD4, SAMPLE_CHANNEL, 550);
  pwmEnableChannelNotification(&PWMD4, SAMPLE_CHANNEL);

  osalSysLock();
  osalThreadSuspendS(&owp->thread);
  osalSysUnlock();

  pwmStop(&PWMD4);
  return owp->slave_present;
}

/**
 *
 */
void onewireWriteData(OWDriver *owp, uint8_t *txbuf, size_t txbytes) {

  owp->txbuf = txbuf;
  owp->txbit = 0;
  owp->txbytes = txbytes;

  pwmStart(&PWMD4, &pwmcfg_tx);
  pwmEnablePeriodicNotification(&PWMD4);

  osalSysLock();
  osalThreadSuspendS(&owp->thread);
  osalSysUnlock();

  pwmStop(&PWMD4);
}

/**
 *
 */
void onewireReadData(OWDriver *owp, uint8_t *rxbuf, size_t rxbytes) {

  /* Buffer zeroing this is important because of driver collects
     bits using |= operation.*/
  memset(rxbuf, 0, rxbytes);

  owp->rxbit = 0;
  owp->rxbuf = rxbuf;
  owp->rxbytes = rxbytes;

  pwmStart(&PWMD4, &pwmcfg_rx);
  pwmEnablePeriodicNotification(&PWMD4);

  osalSysLock();
  osalThreadSuspendS(&owp->thread);
  osalSysUnlock();

  pwmStop(&PWMD4);
}



static const uint8_t crc_table[256] = {
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



static uint8_t get_crc(const uint8_t *buf, size_t len) {
  uint8_t ret = 0;

  for (size_t i=0; i<len; i++)
    ret = crc_table[ret ^ buf[i]];

  return ret;
}


/**
 *
 */
static uint8_t buf[12];
static float temperature;

void onewireTest(void) {

  uint16_t tmp;
  size_t conv_time;
  uint64_t rom;

  onewireObjectInit(&OWD1);
  onewireStart(&OWD1, &ow_cfg);

  while (true) {
    if (true == onewireReset(&OWD1)){
      red_led_on();

//      buf[0] = SEARCH_ROM;
//      onewireWriteData(&OWD1, buf, 1);
//      rom = onewireSearchRom(&OWD1);



      /* test read rom command */
      buf[0] = READ_ROM;
      onewireWriteData(&OWD1, buf, 1);
      onewireReadData(&OWD1, buf, 8);
      osalDbgCheck(buf[7] == get_crc(buf, 7));

      /* use read rom command to fill buffer with device address */
      onewireReset(&OWD1);
      buf[0] = READ_ROM;
      onewireWriteData(&OWD1, buf, 1);
      onewireReadData(&OWD1, &buf[1], 8);

      /* start temperature measurement */
      onewireReset(&OWD1);
      buf[0] = MATCH_ROM;
      buf[9] = CONVERT_TEMPERATURE;
      onewireWriteData(&OWD1, buf, 10);
      buf[0] = 0;
      conv_time = 0;
      while (buf[0] == 0){
        osalThreadSleepMilliseconds(1);
        onewireReadData(&OWD1, buf, 1);
        conv_time++;
      }

      /* acqure temperature from scratchpad */
      onewireReset(&OWD1);
      buf[0] = MATCH_ROM;
      buf[9] = READ_SCRATCHPAD;
      onewireWriteData(&OWD1, buf, 10);
      onewireReadData(&OWD1, buf, 9);
      osalDbgCheck(buf[8] == get_crc(buf, 8));
      tmp |= (buf[1] << 8) | buf[0];
      temperature = tmp * 0.0625;
    }
    else {
      red_led_off();
    }

    osalThreadSleepMilliseconds(1);
  }
}
