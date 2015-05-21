#include <cstring>

#include "main.h"
#include "pads.h"

#include "adis.hpp"
#include "exti_local.hpp"
#include "array_len.hpp"
#include "param_registry.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define ADIS_START_TIME_MS        560
#define ADIS_WAIT_TIMEOUT         MS2ST(200)

/* some address definitions */
#define DIAG_STS                  0x0A
#define GLOB_CMD                  0x02
#define PROD_ID                   0x7E
#define SYS_E_FLAG                0x08
#define EKF_CNFG                  0x50

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

/*
 * Software delay for this stupid sensor ~1.5uS
 */
static const size_t ADIS_NSS_DELAY_US = (STM32_SYSCLK + STM32_SYSCLK / 2) / 1000000;

static const uint16_t ADIS_INTERNAL_SAMPLE_RATE = 2460;

static const uint16_t supported_models[] = {16480};

/*
 * Maximum speed SPI configuration (84MHz/8, CPHA=1, CPOL=1, 16bit, MSb first).
 */
static const SPIConfig spicfg = {
  NULL,
  GPIOA,
  GPIOA_ADIS_NSS,
  SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_DFF // (84MHz/8, CPHA=1, CPOL=1, 16bit, MSb first).
};

chibios_rt::BinarySemaphore Adis::isr_sem(true);

static const uint8_t request[] = {
    0x08, // sys error flags
    0x0E, // temp
    0x10, // GYR
    0x12,
    0x14,
    0x16,
    0x18,
    0x1A,
    0x1C, // ACC
    0x1E,
    0x20,
    0x22,
    0x24,
    0x26,
    0x28, // mag
    0x2A,
    0x2C,
    0x2E,
    0x30,
    0x60, // Q0 (c11)
    0x62, // Q1 (c12)
    0x64, // Q2 (c13)
    0x66, // Q3 (c21)
    0x68, // c22
    0x6A, // roll (c23)
    0x6C, // pitch (c31)
    0x6E, // yaw (c32)
    0x70, // c33
    0x70 /* special fake read for stupid adis logic */
};

static uint16_t rxbuf[ArrayLen(request)];

static const float gyr_scale   = 0.00000000532632218; /* to rad/s */
static const float acc_scale   = 0.00000011975097664; /* to m/s^2 */
static const float mag_scale   = 0.0001; /* to gauss */
static const float baro_scale  = 0.04; /* to millibars */
static const float temp_scale  = 0.00565; /* to celsius */
static const float quat_scale  = 0.000030517578125;
static const float euler_scale = 0.00009587379924285258; /* to rad (2*pi/65536) */

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
static uint16_t read(uint8_t address){
  uint16_t ret;

  osalDbgCheck(address < 128);
  spiSelect(&ADIS_SPI);
  ret = spiPolledExchange(&ADIS_SPI, address << 8);
  spiUnselect(&ADIS_SPI);
  chSysPolledDelayX(ADIS_NSS_DELAY_US);

  return ret;
}

/**
 *
 */
static void write(uint8_t address, uint16_t word) {

  spiSelect(&ADIS_SPI);
  spiPolledExchange(&ADIS_SPI, (1 << 15) | (address << 8) | (word & 0xFF));
  spiUnselect(&ADIS_SPI);
  chSysPolledDelayX(ADIS_NSS_DELAY_US);

  spiSelect(&ADIS_SPI);
  spiPolledExchange(&ADIS_SPI, (1 << 15) | ((address + 1) << 8) | ((word >> 8) & 0xFF));
  spiUnselect(&ADIS_SPI);
  chSysPolledDelayX(ADIS_NSS_DELAY_US);
}

/**
 * @note    This function does not use defined write() because page select
 *          is special case of write - single byte.
 */
static void select_page(uint8_t page) {

  osalDbgCheck(page < 13);
  spiSelect(&ADIS_SPI);
  spiPolledExchange(&ADIS_SPI, (1 << 15) | page);
  spiUnselect(&ADIS_SPI);
  chSysPolledDelayX(ADIS_NSS_DELAY_US);
}

/**
 *
 */
static bool check_id(void){

  uint16_t ack;
  size_t i;

  read(PROD_ID);
  ack = read(PROD_ID);

  for (i=0; i<ArrayLen(supported_models); i++){
    if (supported_models[i] == ack)
      return OSAL_SUCCESS;
  }
  return OSAL_FAILED;
}

/**
 *
 */
static bool selftest(void){

  uint16_t status;

  select_page(3);
  write(GLOB_CMD, 1 << 1);
  osalThreadSleepMilliseconds(12);
  select_page(0);

  status = read(DIAG_STS);
  if (0 == status)
    return OSAL_SUCCESS;
  else
    return OSAL_FAILED;
}

/**
 *
 */
template<typename T>
static T u16_conv(T scale, uint16_t msb){
  return scale * (int16_t)msb;
}

/**
 *
 */
template<typename T>
static T u32_conv(T scale, uint32_t msb, uint32_t lsb) {
  int32_t data = 0;
  data |= (msb << 16) | lsb;
  return scale * data;
}

/**
 *
 */
template<typename T>
static void u16_block_conv(T scale, const uint16_t *raw, T *ret, size_t len){
  for (size_t i = 0; i<len; i++)
    ret[i] = u16_conv(scale, raw[i]);
}

/**
 *
 */
template<typename T>
static void u32_block_conv(T scale, const uint16_t *raw, T *ret, size_t len){
  for (size_t i = 0; i<len; i++)
    ret[i] = u32_conv(scale, raw[2*i+1], raw[2*i]);
}


/**
 *
 */
void Adis::set_lock(void) {
  this->protect_sem.wait();
}

/**
 *
 */
void Adis::release_lock(void) {
  this->protect_sem.signal();
}

/**
 *
 */
bool Adis::hw_init_fast(void) {
  return OSAL_SUCCESS;
}

/**
 *
 */
bool Adis::hw_init_full(void) {
  return OSAL_SUCCESS;
}

/**
 *
 */
void Adis::acquire_data(void) {

  chTMStartMeasurementX(&tm);

  /* reading data */
  read(request[0]); /* first read for warm up */
  for (size_t i=1; i<ArrayLen(request); i++) // NOTE: this loop must be start from #1
    rxbuf[i-1] = read(request[i]);

  /* converting to human useful values */
  this->set_lock();
  measurement.temp = 25 + u16_conv(temp_scale, rxbuf[1]);
  u32_block_conv(acc_scale, &rxbuf[8], measurement.acc, 3);
  u32_block_conv(gyr_scale, &rxbuf[2], measurement.gyr, 3);
  u16_block_conv(mag_scale, &rxbuf[14], measurement.mag, 3);
  measurement.baro = u32_conv(baro_scale, rxbuf[14], rxbuf[15]);
  u16_block_conv(quat_scale, &rxbuf[19], measurement.quat, 4);
  u16_block_conv(euler_scale, &rxbuf[24], measurement.euler, 3);
  measurement.errors = rxbuf[0];
  this->release_lock();

  chTMStopMeasurementX(&tm);
}

/**
 *
 */
void Adis::set_sample_rate(void) {
  select_page(3);
  write(0x0C, smplrtdiv_current - 1);
  select_page(0);
}

/**
 *
 */
void Adis::set_kalman(void) {
  select_page(3);
  write(EKF_CNFG, 1 << 3);
  select_page(0);
}

/**
 *
 */
void Adis::param_update(void) {
  uint32_t s = *smplrtdiv;

  if (s != smplrtdiv_current) {
    set_sample_rate();
    smplrtdiv_current = s;
  }
}

/**
 *
 */
float Adis::dT(void) {
  return smplrtdiv_current / static_cast<float>(ADIS_INTERNAL_SAMPLE_RATE);
}

/**
 *
 */
__CCM__ static THD_WORKING_AREA(AdisThreadWA, 256);
THD_FUNCTION(AdisThread, arg) {
  chRegSetThreadName("Adis");
  Adis *self = static_cast<Adis *>(arg);
  msg_t semstatus = MSG_RESET;

  while (!chThdShouldTerminateX()) {
    semstatus = self->isr_sem.wait(ADIS_WAIT_TIMEOUT);
    if (MSG_OK == semstatus) {
      self->acquire_data();
      self->data_ready_sem.signal();
      self->param_update();
    }
  }

  chThdExit(MSG_OK);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
Adis::Adis(void):
protect_sem(false),
data_ready_sem(true)
{
  state = SENSOR_STATE_STOP;
  chTMObjectInit(&tm);
  return;
}

/**
 *
 */
sensor_state_t Adis::start(void) {

  if (SENSOR_STATE_STOP == this->state) {
    param_registry.valueSearch("ADIS_smplrtdiv", &smplrtdiv);
    smplrtdiv_current = *smplrtdiv;

    adis_reset_clear();
    spiStart(&ADIS_SPI, &spicfg);
    osalThreadSleepMilliseconds(ADIS_START_TIME_MS);

    if (OSAL_SUCCESS != check_id()) {
      this->state = SENSOR_STATE_DEAD;
      return this->state;
    }

    if (OSAL_SUCCESS != selftest()) {
      this->state = SENSOR_STATE_DEAD;
      return this->state;
    }

    set_sample_rate();
    set_kalman();

    Exti.adis(true);

    worker = chThdCreateStatic(AdisThreadWA, sizeof(AdisThreadWA),
                               ADISPRIO, AdisThread, this);
    osalDbgAssert(nullptr != worker, "Can not allocate RAM");

    this->state = SENSOR_STATE_READY;
  }

  return this->state;
}

/**
 *
 */
void Adis::stop(void) {

  if ((SENSOR_STATE_DEAD == this->state) || (SENSOR_STATE_STOP == this->state))
    return;
  else {
    if (SENSOR_STATE_READY == this->state) {
      chThdTerminate(worker);
      isr_sem.reset(false); /* speedup termination */
      chThdWait(worker);
      worker = nullptr;
    }

    adis_reset_assert();
    spiStop(&ADIS_SPI);
    this->state = SENSOR_STATE_STOP;
  }
}

/**
 * @note    In sleep mode Adis remains in page#3
 */
void Adis::sleep(void) {

  if (this->state == SENSOR_STATE_SLEEP)
    return;

  osalDbgAssert(this->state == SENSOR_STATE_READY, "Invalid state");

  /* terminate thread */
  chThdTerminate(worker);
  chThdWait(worker);
  worker = nullptr;

  /* suspend sensor */
  select_page(3);
  /* first we have to write 0 to SLP_CNT bits
     for infinite sleep (not sure if it really need) */
  write(0x10, 0);
  /* now set sleep bit */
  write(0x10, 1 << 8);

  this->state = SENSOR_STATE_SLEEP;
}

/**
 * @note    In sleep mode Adis remains in page#3
 */
sensor_state_t Adis::wakeup(void) {

  if (this->state == SENSOR_STATE_READY)
    return this->state;

  osalDbgAssert(this->state == SENSOR_STATE_SLEEP, "Invalid state");

  /* To wake up adis it is enough to assert CS line low. */
  spiSelect(&ADIS_SPI);
  osalThreadSleepMilliseconds(1); /* sleep recovery time is 700uS */
  select_page(0);

  worker = chThdCreateStatic(AdisThreadWA, sizeof(AdisThreadWA),
                             ADISPRIO, AdisThread, this);
  osalDbgAssert(nullptr != worker, "Can not allocate RAM");

  this->state = SENSOR_STATE_READY;
  return this->state;
}

/**
 *
 */
sensor_state_t Adis::get(ahrs_data_t &result) {

  if (SENSOR_STATE_READY == this->state) {
    set_lock();
    if (1 == result.request.euler)
      memcpy(result.euler, &measurement.euler, sizeof(result.euler));
    if (1 == result.request.quat)
      memcpy(result.quat, &measurement.quat, sizeof(result.quat));
    result.dT = this->dT();
    release_lock();
  }
  return this->state;
}

/**
 *
 */
msg_t Adis::waitData(systime_t timeout) {
  return data_ready_sem.wait(timeout);
}

/**
 *
 */
sensor_state_t Adis::get(marg_data_t &result) {

  if (SENSOR_STATE_READY == this->state) {
    set_lock();
    if (1 == result.request.acc)
      memcpy(result.acc, &measurement.acc, sizeof(result.acc));
    if (1 == result.request.gyr)
      memcpy(result.gyr, &measurement.gyr, sizeof(result.gyr));
    if (1 == result.request.mag)
      memcpy(result.mag, &measurement.mag, sizeof(result.mag));
    if (1 == result.request.dT)
      result.dT = this->dT();
    release_lock();
  }
  return this->state;
}

/**
 *
 */
void Adis::extiISR(EXTDriver *extp, expchannel_t channel){
  (void)extp;
  (void)channel;

  osalSysLockFromISR();
  isr_sem.signalI();
  osalSysUnlockFromISR();
}






