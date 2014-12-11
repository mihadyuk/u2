#include "main.h"
#include "lsm303_mag.hpp"
#include "pack_unpack.h"
#include "param_registry.hpp"
#include "exti_local.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/* Swaps Y Z axis for specially LSM303DLHC. LSM303DLH and HMC5843 does not need
   this hack */
#define LSM_SWAP_Y_Z          TRUE

#define LSM_REG_CRA           0x00
#define LSM_REG_CRB           0x01
#define LSM_REG_MR            0x02
#define LSM_REG_MAG_OUT       0x03
#define LSM_REG_ID            0x0A
#define LSM_REG_TEMP_OUT      0x31
#define GAIN_BITS_SHIFT       5

/**
 * @brief   Magnetometer gain (LSB/Gauss)
 */
typedef enum {
  LSM_MAG_GAIN_1370 = 0,
  LSM_MAG_GAIN_1090,
  LSM_MAG_GAIN_820,
  LSM_MAG_GAIN_660,
  LSM_MAG_GAIN_440,
  LSM_MAG_GAIN_390,
  LSM_MAG_GAIN_330,
  LSM_MAG_GAIN_230
} mag_sens_t;

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

static const float mag_sens_array[8] = {
    1.0f / 1370,
    1.0f / 1090,
    1.0f / 820,
    1.0f / 660,
    1.0f / 440,
    1.0f / 390,
    1.0f / 330,
    1.0f / 230
};

static bool mag_data_fresh = true;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 * @brief   convert parrots to Gauss.
 */
float LSM303_mag::mag_sens(void) {
  return mag_sens_array[*gain];
}

/**
 *
 */
void LSM303_mag::thermo_comp(float *result){
  (void)result;
}

/**
 *
 */
void LSM303_mag::iron_comp(float *result){
  (void)result;
}

/**
 *
 */
void LSM303_mag::pickle(float *result, int16_t *result_raw) {

  int16_t raw[3];
  float sens = this->mag_sens();

  for (size_t i=0; i<3; i++)
    raw[i] = static_cast<int16_t>(pack8to16be(&rxbuf[i*2]));

  #if LSM_SWAP_Y_Z
  float tmp = raw[2];
  raw[2] = raw[1];
  raw[1] = tmp;
  #endif

  /* */
  for (size_t i=0; i<3; i++) {
    result[i] = sens * raw[i];
    result_raw[i] = raw[i];
  }
  thermo_comp(result);
  iron_comp(result);
}

/**
 *
 */
bool LSM303_mag::hw_init_fast(void){
  return hw_init_full(); /* unimplemented */
}

/**
 *
 */
bool LSM303_mag::hw_init_full(void){

  msg_t i2cret = MSG_RESET;

  txbuf[0] = LSM_REG_ID;
  i2cret = transmit(txbuf, 1, rxbuf, 3);
  if (MSG_OK != i2cret)
    return OSAL_FAILED;
  if(0 != memcmp("H43", rxbuf, 3)) // incorrect ID
    return OSAL_FAILED;

  txbuf[0] = LSM_REG_CRA;
  //txbuf[1] = 0b10011100; /* enable thermometer and set maximum output rate */
  txbuf[1] = 0;
  /* Set gain. 001 is documented for LSM303 and 000 is for HMC5883.
   * 000 looks working for LSM303 too - lets use it. */
  txbuf[2] = *gain << GAIN_BITS_SHIFT;
  /* single conversion mode */
  txbuf[3] = 0b00000001;

  i2cret = transmit(txbuf, 4, NULL, 0);
  if (MSG_OK != i2cret)
    return OSAL_FAILED;

  Exti.lsm303(true);

  return OSAL_SUCCESS;
}

/**
 *
 */
msg_t LSM303_mag::start_single_measurement(void) {
  txbuf[0] = LSM_REG_MR;
  txbuf[1] = 0b00000001;
  return transmit(txbuf, 2, NULL, 0);
}

/**
 *
 */
msg_t LSM303_mag::set_gain(uint8_t val) {
  txbuf[0] = LSM_REG_CRB;
  txbuf[1] = val;
  return transmit(txbuf, 2, NULL, 0);
}

/**
 *
 */
msg_t LSM303_mag::refresh_gain(void) {

  uint8_t tmp = *gain;

  if (tmp != gain_prev) {
    gain_prev = tmp;
    return set_gain(tmp << GAIN_BITS_SHIFT);
  }
  else
    return MSG_OK;
}

/**
 *
 */
msg_t LSM303_mag::get_prev_measurement(float *result, int16_t *result_raw) {

  msg_t ret = MSG_RESET;

  txbuf[0] = LSM_REG_MAG_OUT;
  ret = transmit(txbuf, 1, rxbuf, 6);
  if (MSG_OK == ret) {
    pickle(result, result_raw);
    memcpy(cache, result, sizeof(cache));
    memcpy(cache_raw, result_raw, sizeof(cache_raw));
  }

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
LSM303_mag::LSM303_mag(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr),
sample_cnt(0)
{
  state = SENSOR_STATE_STOP;
}

/**
 *
 */
sensor_state_t LSM303_mag::get(float *result, int16_t *result_raw) {

  if (SENSOR_STATE_READY == this->state) {
    osalDbgCheck((nullptr != result) && (nullptr != result_raw));
    if (true == mag_data_fresh) {
      if (MSG_OK != get_prev_measurement(result, result_raw))
        this->state = SENSOR_STATE_DEAD;
      mag_data_fresh = false;
      if (MSG_OK != start_single_measurement())
        this->state = SENSOR_STATE_DEAD;
    }
    else {
      memcpy(result, cache, sizeof(cache));
      memcpy(result_raw, cache_raw, sizeof(cache_raw));
    }

    if (MSG_OK != refresh_gain())
      this->state = SENSOR_STATE_DEAD;
  }

  return this->state;
}

/**
 *
 */
sensor_state_t LSM303_mag::start(void){

  if (SENSOR_STATE_STOP == this->state) {
    param_registry.valueSearch("LSMM_gain",  &gain);
    gain_prev = *gain;

    bool ret;
    if (need_full_init())
      ret = hw_init_full();
    else
      ret = hw_init_fast();

    if (OSAL_SUCCESS != ret)
      this->state = SENSOR_STATE_DEAD;
    else
      this->state = SENSOR_STATE_READY;
  }

  return this->state;
}

/**
 *
 */
msg_t LSM303_mag::stop_sleep_code(void) {
  txbuf[0] = LSM_REG_MR;
  txbuf[1] = 0b00000011;
  return transmit(txbuf, 2, NULL, 0);
}

/**
 *
 */
void LSM303_mag::stop(void){
  if (this->state == SENSOR_STATE_STOP)
    return;

  osalDbgAssert(this->state == SENSOR_STATE_READY, "Invalid state");
  if (MSG_OK != stop_sleep_code())
    this->state = SENSOR_STATE_DEAD;
  else
    this->state = SENSOR_STATE_STOP;
}

/**
 *
 */
void LSM303_mag::sleep(void) {
  if (this->state == SENSOR_STATE_SLEEP)
    return;

  osalDbgAssert(this->state == SENSOR_STATE_READY, "Invalid state");
  if (MSG_OK != stop_sleep_code())
    this->state = SENSOR_STATE_DEAD;
  else
    this->state = SENSOR_STATE_SLEEP;
}

/**
 *
 */
sensor_state_t LSM303_mag::wakeup(void) {
  if (this->state == SENSOR_STATE_READY)
    return this->state;

  osalDbgAssert(this->state == SENSOR_STATE_SLEEP, "Invalid state");
  this->state = SENSOR_STATE_READY;
  return this->state;
}

/**
 *
 */
void LSM303_mag::extiISR(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  mag_data_fresh = true;
}
