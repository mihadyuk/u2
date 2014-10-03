#include "main.h"

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

static bool slave_present = false;

static void pwmc4_presence_cb(PWMDriver *pwmp) {
  (void)pwmp;

  osalSysLockFromISR();
  slave_present = (PAL_LOW == palReadPad(GPIOB, GPIOB_TACHOMETER));
  osalSysUnlockFromISR();
}

static const PWMConfig pwmcfg_reset = {
  1000000,
  960,
  NULL,
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, pwmc4_presence_cb}
  },
  0,
  0
};





#define ZERO_WIDTH        60
#define ONE_WIDTH         6
#define SAMPLE_WIDTH      15

#define MASTER_CHANNEL    2
#define SAMPLE_CHANNEL    3


static uint8_t tx_data;
static size_t bit = 0;
static uint8_t tmp = 0;

static void pwmpcb_tx(PWMDriver *pwmp) {

  osalSysLockFromISR();

  if (0 == bit){
    tmp = tx_data;
    palClearPad(GPIOB, GPIOB_LED_R);
  }

  if (8 == bit){
    //palSetPad(GPIOB, GPIOB_LED_R);
    pwmDisableChannelI(pwmp, MASTER_CHANNEL);
    pwm_lld_disable_periodic_notification(pwmp);
    bit = 0;
    osalSysUnlockFromISR();
    return;
  }

  if ((tmp & (1 << bit)) > 0)
    pwmEnableChannelI(pwmp, MASTER_CHANNEL, ONE_WIDTH);
  else
    pwmEnableChannelI(pwmp, MASTER_CHANNEL, ZERO_WIDTH);

  bit++;

  osalSysUnlockFromISR();
}

static const PWMConfig pwmcfg_tx = {
  1000000,                                    /* PWM clock frequency.   */
  70,                                    /* Initial PWM period        */
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

static void pwmpcb_rx(PWMDriver *pwmp) {

  osalSysLockFromISR();

  bit = 0;
  pwmEnableChannelI(pwmp, MASTER_CHANNEL, ONE_WIDTH);
  pwmEnableChannelI(pwmp, SAMPLE_CHANNEL, SAMPLE_WIDTH);
  pwm_lld_disable_periodic_notification(pwmp);
  pwm_lld_enable_channel_notification(pwmp, SAMPLE_CHANNEL);

  osalSysUnlockFromISR();
}

static uint8_t rx_data;

static void pwmc4_read_cb(PWMDriver *pwmp) {

  osalSysLockFromISR();
  palTogglePad(GPIOB, GPIOB_LED_R);
  rx_data |= palReadPad(GPIOB, GPIOB_TACHOMETER) << bit;
  bit++;
  if (bit == 8){
    pwmDisableChannelI(pwmp, MASTER_CHANNEL);
    pwmDisableChannelI(pwmp, SAMPLE_CHANNEL);
    pwm_lld_disable_channel_notification(pwmp, SAMPLE_CHANNEL);
    bit = 0;
  }

  osalSysUnlockFromISR();
}

static const PWMConfig pwmcfg_rx = {
  1000000,                                    /* PWM clock frequency.   */
  70,                                    /* Initial PWM period        */
  pwmpcb_rx,
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, pwmc4_read_cb}
  },
  0,
  0
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

static bool onewireReset(void){

  size_t retry = 5;
  slave_present = false;

  pwmStart(&PWMD4, &pwmcfg_reset);
  //pwmEnablePeriodicNotification(&PWMD4);
  //osalThreadSleepMilliseconds(100);
  pwmEnableChannel(&PWMD4, MASTER_CHANNEL, 480);
  //pwmEnableChannelNotification(&PWMD4, MASTER_CHANNEL);

  pwmEnableChannel(&PWMD4, SAMPLE_CHANNEL, 550);
  pwmEnableChannelNotification(&PWMD4, SAMPLE_CHANNEL);

  while (retry--) {
    osalThreadSleepMilliseconds(1);
    if (true == slave_present)
      break;
  }

  pwmStop(&PWMD4);
  return slave_present;
}

static void onewireSendByte(uint8_t data) {
  tx_data = data;
  pwmEnablePeriodicNotification(&PWMD4);
}

static void onewireGetByte(void) {
  pwmEnablePeriodicNotification(&PWMD4);
}

void onewireObjectInit(void){

  if (true == onewireReset()) {

    //pwmEnablePeriodicNotification(&PWMD4);
    //osalThreadSleepMilliseconds(100);
    //pwmEnableChannel(&PWMD4, MASTER_CHANNEL, 480);
    //pwmEnableChannelNotification(&PWMD4, MASTER_CHANNEL);

    //pwmEnableChannel(&PWMD4, SAMPLE_CHANNEL, 550);
    //pwmEnableChannelNotification(&PWMD4, SAMPLE_CHANNEL);

    while (1) {
      osalThreadSleepMilliseconds(2);

      pwmStart(&PWMD4, &pwmcfg_tx);
      osalThreadSleepMilliseconds(2);

      onewireSendByte(0x33);
      osalThreadSleepMilliseconds(2);
      pwmStop(&PWMD4);

      osalThreadSleepMilliseconds(2);
      pwmStart(&PWMD4, &pwmcfg_rx);
      osalThreadSleepMilliseconds(2);

      onewireGetByte();
      osalThreadSleepMilliseconds(2);
      rx_data = 0;
      onewireGetByte();
      osalThreadSleepMilliseconds(2);
      rx_data = 0;
      onewireGetByte();
      osalThreadSleepMilliseconds(2);
      rx_data = 0;
      onewireGetByte();
      osalThreadSleepMilliseconds(2);
      rx_data = 0;
      onewireGetByte();
      osalThreadSleepMilliseconds(2);
      rx_data = 0;
      onewireGetByte();
      osalThreadSleepMilliseconds(2);
      rx_data = 0;
      onewireGetByte();
      osalThreadSleepMilliseconds(2);
      rx_data = 0;
      onewireGetByte();
      osalThreadSleepMilliseconds(2);

      pwmStop(&PWMD4);
    }
  }
  else
    osalSysHalt("");
}


