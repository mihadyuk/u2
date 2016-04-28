#pragma GCC optimize "-O2"

#include "main.h"
#include "pads.h"

#include "exti_local.hpp"
#include "mpu6050.hpp"
#include "pack_unpack.h"
#include "geometry.hpp"
#include "param_registry.hpp"

#include "mpu6050_fir_taps.h"
#include "array_len.hpp"
#include "putinrange.hpp"
#include "mav_logger.hpp"
#include "mav_postman.hpp"
#include "mav_dbg_sender.hpp"
#include "debug_indices.h"
#include "polynomial.hpp"
#include <cstdio>

using namespace filters;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/* offsets in received data array */
#define MPU_ACCEL_OFFSET        1
#define MPU_TEMP_OFFSET         7
#define MPU_GYRO_OFFSET         9

/* registers address */
#define MPUREG_SMPLRT_DIV       0x19
#define MPUREG_CONFIG           0x1A
#define MPUREG_GYRO_CONFIG      0x1B
#define MPUREG_ACCEL_CONFIG     0x1C
#define MPUREG_FIFO_EN          0x23
  #define FIFO_DATA_BITS        0b1111000 /* accel and gyro */
  #define FIFO_MODE             (1 << 6)
#define MPUREG_INT_PIN_CFG      0x37
  #define I2C_BYPASS_EN         (1 << 1)
  #define INT_RD_CLEAR          (1 << 4) /* clear int flag in register on any read operation */
#define MPUREG_INT_ENABLE       0x38
#define MPUREG_INT_STATUS       0x3A /* 1 status bite and 14 bytes of data */
#define MPUREG_TEMP_OUT         0x41  /* MSB. Next byte is LSB */
#define MPUREG_USER_CTRL        0x6A
  #define FIFO_EN               (1 << 6)
  #define I2C_MST_EN            (1 << 5) /* clear this bit to use I2C bypass mode */
  #define FIFO_RST              (1 << 2)
#define MPUREG_PWR_MGMT1        0x6B
  #define DEVICE_RESET          (1 << 7)
  #define DEVICE_SLEEP          (1 << 6)
#define MPUREG_PWR_MGMT2        0x6C
#define MPUREG_FIFO_CNT         0x72 /* MSB. Next byte is LSB */
#define MPUREG_FIFO_DATA        0x74
#define MPUREG_WHO_AM_I         0x75
  #define WHO_AM_I_VAL          0X68

/**
 * @brief   Human readable termo compensation subtypes
 */
enum class tcomp_t {
  BIAS,
  SENS
};

/* reset fifo if it contains such amount of bytes */
#define FIFO_RESET_THRESHOLD  1024

/* how many bytes in single fifo sample */
#define BYTES_IN_SAMPLE       12

#define MPU6050_USE_IIR       true

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static const float gyro_sens_array[4] = {
    deg2rad(250.0f  / 32768),
    deg2rad(500.0f  / 32768),
    deg2rad(1000.0f / 32768),
    deg2rad(2000.0f / 32768)
};

static const float acc_sens_array[4] = {
    (2  * 9.81f) / 32768,
    (4  * 9.81f) / 32768,
    (8  * 9.81f) / 32768,
    (16 * 9.81f) / 32768
};

/* IIR taps -40dB on 3 Hz */
static const float iir_taps_a1[IIR_LEN] = {
    1.9825484752655029296875,  -0.982643544673919677734375};

static const float iir_taps_a2[IIR_LEN] = {
    1.994026660919189453125,   -0.994111359119415283203125};

static const float *iir_taps_a[IIR_SEC] = {iir_taps_a1, iir_taps_a2};


static const float iir_taps_b1[IIR_LEN+1] = {
    1,  -1.997833728790283203125,   1};

static const float iir_taps_b2[IIR_LEN+1] = {
    1,  -1.9996240139007568359375,  1};

static const float *iir_taps_b[IIR_SEC] = {iir_taps_b1, iir_taps_b2};

static const float gain[IIR_SEC] ={
    0.22409628331661224365234375,
    0.044132225215435028076171875};

__CCM__ static MPU6050_iir_block<float> iir_block(iir_taps_a, iir_taps_b, gain);

__CCM__ static MPU6050_fir_block<float> fir_block(taps);

size_t MPU6050::isr_count = 0;
uint8_t MPU6050::isr_dlpf = 0;
uint8_t MPU6050::isr_smplrtdiv = 0;
chibios_rt::BinarySemaphore MPU6050::isr_sem(true);

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/**
 *
 */
float MPU6050::gyr_sens(void) {
  return gyro_sens_array[gyr_fs_current];
}

/**
 *
 */
float MPU6050::acc_sens(void) {
  return acc_sens_array[acc_fs_current];
}

/**
 *
 */
static void pickle_temp(float *result, const uint8_t *buf) {
  result[0] = static_cast<int16_t>(pack8to16be(buf));
  result[0] /= 340;
  result[0] += 36.53f;
}

/**
 *
 */
static void thermo_comp(marg_vector_t &result, const float **coeff_ptr,
                        tcomp_t type, float temperature) {
  size_t axis, i;
  float poly_c[POLYC_LEN];

  for (axis=0; axis<3; axis++) {
    const size_t tip = 3*axis + (POLYC_LEN-1);
    for (i=0; i<POLYC_LEN; i++) {
      poly_c[i] = *coeff_ptr[tip - i]; //x^2 goes first
    }

    switch(type) {
    case tcomp_t::BIAS:
      result[axis] -= PolyMul(poly_c, POLYC_LEN, temperature);
      break;
    case tcomp_t::SENS:
      result[axis] *= PolyMul(poly_c, POLYC_LEN, temperature);
      break;
    default:
      osalSysHalt("Unhandled type");
      break;
    }
  }
}

/**
 *
 */
void MPU6050::pickle_gyr(marg_vector_t &result) {

  int16_t raw[3];
  uint8_t *b = &rxbuf[MPU_GYRO_OFFSET];
  float sens = this->gyr_sens();

  toggle_endiannes16(b, 6);
  memcpy(raw, b, sizeof(raw));

  for (size_t i=0; i<3; i++) {
    gyr_raw_data[i] = raw[i];
    result[i] = sens * raw[i];
  }
  if (*MPUG_Tcomp_en != 0) {
    thermo_comp(result, gyr_bias_c, tcomp_t::BIAS, temperature);
  }
}

/**
 *
 */
void MPU6050::pickle_acc(marg_vector_t &result) {

  int16_t raw[3];
  uint8_t *b = &rxbuf[MPU_ACCEL_OFFSET];
  float sens = this->acc_sens();

  raw[0] = static_cast<int16_t>(pack8to16be(&b[0]));
  raw[1] = static_cast<int16_t>(pack8to16be(&b[2]));
  raw[2] = static_cast<int16_t>(pack8to16be(&b[4]));

  for (size_t i=0; i<3; i++) {
    acc_raw_data[i] = raw[i];
    result[i] = sens * raw[i];
  }
  if (*MPUA_Tcomp_en != 0) {
    thermo_comp(result, acc_bias_c, tcomp_t::BIAS, temperature);
    thermo_comp(result, acc_sens_c, tcomp_t::SENS, temperature);
  }
}

/**
 *
 */
bool MPU6050::hw_init_fast(void) {
  return hw_init_full(); /* unimplemented */
}

/**
 *
 */
msg_t MPU6050::set_gyr_fs(uint8_t fs) {
  txbuf[0] = MPUREG_GYRO_CONFIG;
  txbuf[1] = fs << 3;
  return transmit(txbuf, 2, NULL, 0);
}

/**
 *
 */
msg_t MPU6050::set_acc_fs(uint8_t fs) {
  txbuf[0] = MPUREG_ACCEL_CONFIG;
  txbuf[1] = fs << 3;
  return transmit(txbuf, 2, NULL, 0);
}

/**
 *
 */
msg_t MPU6050::set_dlpf_smplrt(uint8_t lpf, uint8_t smplrt) {

  msg_t ret1 = MSG_OK;
  msg_t ret2 = MSG_OK;

  /* */
  txbuf[0] = MPUREG_SMPLRT_DIV;
  if (lpf > 0){
    /* sample rate. If (LPF > 0): (1000 / (val + 1))
     *                      else: (8000 / (val + 1)) */
    txbuf[1] = smplrt - 1; /* val */
    /*    Bandwidth   Delay
    DLPF      (Hz)    (ms)
    1         188     1.9
    2         98      2.8
    3         42      4.8
    4         20      8.3
    5         10      13.4
    6         5       18.6
    7   reserved*/
    txbuf[2] = lpf | FIFO_MODE; /* LPF */
  }
  else{
    txbuf[1] = 7; /* 8000 / (val + 1) */
    txbuf[2] = 0 | FIFO_MODE; /* LPF */
  }
  if (MSG_OK != transmit(txbuf, 3, NULL, 0))
    return MSG_RESET;

  /* FIFO settings */
  if (lpf > 0){
    txbuf[0] = MPUREG_FIFO_EN;
    txbuf[1] = 0;
    ret1 = transmit(txbuf, 2, NULL, 0);

    txbuf[0] = MPUREG_USER_CTRL;
    txbuf[1] = FIFO_RST;
    ret2 = transmit(txbuf, 2, NULL, 0);
  }
  else {
    txbuf[0] = MPUREG_FIFO_EN;
    txbuf[1] = FIFO_DATA_BITS;
    ret1 = transmit(txbuf, 2, NULL, 0);

    txbuf[0] = MPUREG_USER_CTRL;
    txbuf[1] = FIFO_EN;
    ret2 = transmit(txbuf, 2, NULL, 0);
  }

  /**/
  if ((MSG_OK == ret1) && (MSG_OK == ret2))
    return MSG_OK;
  else
    return MSG_RESET;
}

/**
 *
 */
msg_t MPU6050::soft_reset(void) {
  txbuf[0] = MPUREG_PWR_MGMT1;
  txbuf[1] = DEVICE_RESET; /* soft reset */
  return transmit(txbuf, 2, NULL, 0);
}

/**
 *
 */
bool MPU6050::hw_init_full(void){

  msg_t i2c_status = MSG_RESET;

  /* there is not need to call soft reset here because of POR sensor reset,
     just wait timeout. */
  osalThreadSleepMilliseconds(30);

  txbuf[0] = MPUREG_WHO_AM_I;
  i2c_status = transmit(txbuf, 1, rxbuf, 1);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;
  if(WHO_AM_I_VAL != rxbuf[0]) // MPU6050 wrong id
    return OSAL_FAILED;
  osalThreadSleepMilliseconds(1);

  txbuf[0] = MPUREG_PWR_MGMT1;
  txbuf[1] = 1; /* select X gyro as clock source */
  i2c_status = transmit(txbuf, 2, NULL, 0);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;
  osalThreadSleepMilliseconds(5);

  i2c_status = set_gyr_fs(gyr_fs_current);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;
  osalThreadSleepMilliseconds(1);

  i2c_status = set_acc_fs(acc_fs_current);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;
  osalThreadSleepMilliseconds(1);

  txbuf[0] = MPUREG_INT_PIN_CFG;
  txbuf[1] = INT_RD_CLEAR | I2C_BYPASS_EN;
  i2c_status = transmit(txbuf, 2, NULL, 0);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;
  osalThreadSleepMilliseconds(1);

  i2c_status = set_dlpf_smplrt(dlpf_current, smplrtdiv_current);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;
  osalThreadSleepMilliseconds(1);

  txbuf[0] = MPUREG_INT_ENABLE;
  txbuf[1] = 1; /* enable data ready interrupts */
  i2c_status = transmit(txbuf, 2, NULL, 0);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;

  return OSAL_SUCCESS;
}

/**
 *
 */
msg_t MPU6050::param_update(void) {

  uint8_t afs, gfs, lpf, smplrt;

  msg_t ret1 = MSG_OK;
  msg_t ret2 = MSG_OK;
  msg_t ret3 = MSG_OK;

  osalSysLock();
  afs = *acc_fs;
  gfs = *gyr_fs;
  lpf = *dlpf;
  smplrt = *smplrtdiv;
  osalSysUnlock();

  /* gyr full scale */
  if (gfs != gyr_fs_current){
    ret1 = set_gyr_fs(gfs);
    gyr_fs_current = gfs;
  }

  /* acc full scale */
  if (afs != acc_fs_current){
    ret2 = set_acc_fs(afs);
    acc_fs_current = afs;
  }

  /* low pass filter and sample rate */
  if ((lpf != dlpf_current) || (smplrt != smplrtdiv_current)){
    ret3 = set_dlpf_smplrt(lpf, smplrt);
    dlpf_current = lpf;
    smplrtdiv_current = smplrt;
  }

  if ((MSG_OK == ret1) && (MSG_OK == ret2) && (MSG_OK == ret3))
    return MSG_OK;
  else
    return MSG_RESET;
}

/**
 *
 */
msg_t MPU6050::acquire_simple(marg_vector_t &acc, marg_vector_t &gyr) {

  msg_t ret = MSG_RESET;

  txbuf[0] = MPUREG_INT_STATUS;
  ret = transmit(txbuf, 1, rxbuf, sizeof(rxbuf));

  this->set_lock();

  pickle_temp(&temperature, &rxbuf[MPU_TEMP_OFFSET]);
  pickle_gyr(gyr);
  pickle_acc(acc);
  fifo_remainder = 0;

  this->release_lock();

  return ret;
}

/**
 *
 */
void MPU6050::pickle_fifo(marg_vector_t &acc, marg_vector_t &gyr, const size_t sample_cnt) {

  float sens;
  const size_t acc_fifo_offset = 0;
  const size_t gyr_fifo_offset = 3;

  for (size_t i=0; i<3; i++) {
    acc_raw_data[i] = rxbuf_fifo[acc_fifo_offset + i];
    gyr_raw_data[i] = rxbuf_fifo[gyr_fifo_offset + i];
  }

  for (size_t n=0; n<sample_cnt; n++) {
    for (size_t i=0; i<3; i++) {
      size_t shift = n * BYTES_IN_SAMPLE / 2 + i;
      if (MPU6050_USE_IIR) {
        acc[i] = iir.acc[i].update(rxbuf_fifo[shift + acc_fifo_offset]);
        gyr[i] = iir.gyr[i].update(rxbuf_fifo[shift + gyr_fifo_offset]);
      }
      else {
        acc[i] = fir.acc[i].update(rxbuf_fifo[shift + acc_fifo_offset]);
        gyr[i] = fir.gyr[i].update(rxbuf_fifo[shift + gyr_fifo_offset]);
      }
    }
  }

  /* acc */
  sens = this->acc_sens();
  acc[0] *= sens;
  acc[1] *= sens;
  acc[2] *= sens;
  if (*MPUA_Tcomp_en !=0 ) {
    thermo_comp(acc, acc_bias_c, tcomp_t::BIAS, temperature);
    thermo_comp(acc, acc_sens_c, tcomp_t::SENS, temperature);
  }

  /* gyr */
  sens = this->gyr_sens();
  gyr[0] *= sens;
  gyr[1] *= sens;
  gyr[2] *= sens;
  if (*MPUG_Tcomp_en != 0) {
    thermo_comp(gyr, gyr_bias_c, tcomp_t::BIAS, temperature);
  }
}

/**
 *
 */
msg_t MPU6050::acquire_fifo(marg_vector_t &acc, marg_vector_t &gyr) {

  msg_t ret = MSG_RESET;
  size_t recvd;

  txbuf[0] = MPUREG_FIFO_CNT;
  ret = transmit(txbuf, 1, rxbuf, 2);

  recvd = pack8to16be(rxbuf);
  if (recvd >= FIFO_RESET_THRESHOLD) {
    txbuf[0] = MPUREG_USER_CTRL;
    txbuf[1] = FIFO_RST | FIFO_EN;
    return transmit(txbuf, 2, NULL, 0);
  }
  else {
    recvd = putinrange(recvd, 0, sizeof(rxbuf_fifo));
    recvd = (recvd / BYTES_IN_SAMPLE) * BYTES_IN_SAMPLE;

    /* read Acc and Gyr from FIFO */
    txbuf[0] = MPUREG_FIFO_DATA;
    ret = transmit(txbuf, 1, (uint8_t*)rxbuf_fifo, recvd);
    toggle_endiannes16((uint8_t*)rxbuf_fifo, recvd);

    /* read temperature */
    txbuf[0] = MPUREG_INT_STATUS;
    transmit(txbuf, 1, rxbuf, sizeof(rxbuf));

    /* process acquired data */
    this->set_lock();
    pickle_temp(&temperature, &rxbuf[MPU_TEMP_OFFSET]);
    pickle_fifo(acc, gyr, recvd/BYTES_IN_SAMPLE);
    this->release_lock();
  }

  return ret;
}

/**
 *
 */
void MPU6050::acquire_data(void) {

  msg_t ret1 = MSG_RESET;
  msg_t ret2 = MSG_RESET;

  if (SENSOR_STATE_READY == this->state) {
    ret2 = param_update();

    if (dlpf_current > 0) {
      ret1 = acquire_simple(acc_data, gyr_data);
    }
    else {
      ret1 = acquire_fifo(acc_data, gyr_data);
    }

    mav_dbg_sender.send(this->temperature, DEBUG_INDEX_MPU6050, TIME_BOOT_MS);

    if ((MSG_OK != ret1) || (MSG_OK != ret2))
      this->state = SENSOR_STATE_DEAD;
  }
}

/**
 *
 */
void MPU6050::set_lock(void) {
  this->protect_sem.wait();
}

/**
 *
 */
void MPU6050::release_lock(void) {
  this->protect_sem.signal();
}

/**
 *
 */
static THD_WORKING_AREA(Mpu6050ThreadWA, 320);
THD_FUNCTION(Mpu6050Thread, arg) {
  chRegSetThreadName("Mpu6050");
  MPU6050 *self = static_cast<MPU6050 *>(arg);
  msg_t sem_status = MSG_RESET;

  while (!chThdShouldTerminateX()) {
    sem_status = self->isr_sem.wait(MS2ST(self->smplrtdiv_current * 2));
    if (MSG_OK == sem_status) {
      self->isr_dlpf = self->dlpf_current;
      self->isr_smplrtdiv = self->smplrtdiv_current;
      self->acquire_data();
      self->data_ready_sem.signal();
    }
  }

  chThdExit(MSG_OK);
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
MPU6050::MPU6050(I2CDriver *i2cdp, i2caddr_t addr) :
I2CSensor(i2cdp, addr),
protect_sem(false),
data_ready_sem(true),
fir(fir_block),
iir(iir_block)
{
  state = SENSOR_STATE_STOP;
}

/**
 *
 */
sensor_state_t MPU6050::start(void) {

  if (SENSOR_STATE_STOP == this->state) {

    param_registry.valueSearch("MPU_gyr_fs",    &gyr_fs);
    param_registry.valueSearch("MPU_acc_fs",    &acc_fs);
    param_registry.valueSearch("MPU_fir_f",     &fir_f);
    param_registry.valueSearch("MPU_dlpf",      &dlpf);
    param_registry.valueSearch("MPU_smplrtdiv", &smplrtdiv);
    param_registry.valueSearch("MPUG_Tcomp_en", &MPUG_Tcomp_en);
    param_registry.valueSearch("MPUA_Tcomp_en", &MPUA_Tcomp_en);

    char search_key[PARAM_REGISTRY_ID_SIZE];
    for (size_t axis=0; axis<3; axis++) {
      for (size_t i=0; i<POLYC_LEN; i++) {
        snprintf(search_key, PARAM_REGISTRY_ID_SIZE, "MPUG_%cbias_c%u", 'x'+axis, i);
        param_registry.valueSearch(search_key, &gyr_bias_c[3*axis+i]);
        snprintf(search_key, PARAM_REGISTRY_ID_SIZE, "MPUA_%cbias_c%u", 'x'+axis, i);
        param_registry.valueSearch(search_key, &acc_bias_c[3*axis+i]);
        snprintf(search_key, PARAM_REGISTRY_ID_SIZE, "MPUA_%csens_c%u", 'x'+axis, i);
        param_registry.valueSearch(search_key, &acc_sens_c[3*axis+i]);
      }
    }

    osalSysLock();
    gyr_fs_current      = *gyr_fs;
    acc_fs_current      = *acc_fs;
    dlpf_current        = *dlpf;
    smplrtdiv_current   = *smplrtdiv;
    this->isr_dlpf      = *dlpf;
    this->isr_smplrtdiv = *smplrtdiv;
    osalSysUnlock();

    /* init hardware */
    bool init_status = OSAL_FAILED;
    mpu6050_power_on();
    if (need_full_init())
      init_status = hw_init_full();
    else
      init_status = hw_init_fast();

    /* check state */
    osalDbgCheck(OSAL_SUCCESS == init_status);

    if (OSAL_SUCCESS == init_status) {
      worker = chThdCreateStatic(Mpu6050ThreadWA, sizeof(Mpu6050ThreadWA),
                                        MPU6050PRIO, Mpu6050Thread, this);
      osalDbgAssert(nullptr != worker, "Can not allocate RAM");
      Exti.mpu6050(true);
      this->state = SENSOR_STATE_READY;
    }
    else {
      mpu6050_power_off();
      this->state = SENSOR_STATE_DEAD;
    }
  }

  return this->state;
}

/**
 * Just fire soft reset signal. After completing of reset sequence
 * device will be in sleep state.
 */
void MPU6050::stop(void) {

  if ((SENSOR_STATE_READY == this->state) || (SENSOR_STATE_SLEEP == this->state)) {
    if (SENSOR_STATE_READY == this->state) {
      chThdTerminate(worker);
      chThdWait(worker);
      worker = nullptr;
    }

    Exti.mpu6050(false);

    txbuf[0] = MPUREG_PWR_MGMT1;
    txbuf[1] = DEVICE_RESET; /* soft reset */
    if (MSG_OK != transmit(txbuf, 2, NULL, 0))
      this->state = SENSOR_STATE_DEAD;
    else
      this->state = SENSOR_STATE_STOP;

    mpu6050_power_off();
  }
}

/**
 *
 */
sensor_state_t MPU6050::get(marg_data_t &result) {

  if (SENSOR_STATE_READY == this->state) {

    set_lock();
    if (1 == result.request.acc) {
      result.acc     = this->acc_data;
      result.acc_raw = this->acc_raw_data;
    }
    if (1 == result.request.gyr) {
      result.gyr     = this->gyr_data;
      result.gyr_raw = this->gyr_raw_data;
    }
    if (1 == result.request.dT) {
      result.dT = this->dT();
    }
    release_lock();
  }

  return this->state;
}

/**
 *
 */
msg_t MPU6050::waitData(systime_t timeout) {
  return data_ready_sem.wait(timeout);
}

/**
 *
 */
void MPU6050::sleep(void) {
  uint8_t b;

  if (this->state == SENSOR_STATE_SLEEP)
    return;

  osalDbgAssert(this->state == SENSOR_STATE_READY, "Invalid state");

  /* stop worker thread */
  chThdTerminate(worker);
  chThdWait(worker);
  worker = nullptr;

  /* suspend sensor */
  txbuf[0] = MPUREG_PWR_MGMT1;
  if (MSG_OK != transmit(txbuf, 1, &b, 1))
    goto ERROR;
  txbuf[0] = MPUREG_PWR_MGMT1;
  txbuf[1] = b | DEVICE_SLEEP;
  if (MSG_OK != transmit(txbuf, 2, NULL, 0))
    goto ERROR;

  this->state = SENSOR_STATE_SLEEP;
  return;
ERROR:
  this->state = SENSOR_STATE_DEAD;
}

/**
 *
 */
sensor_state_t MPU6050::wakeup(void) {
  uint8_t b;

  if (this->state == SENSOR_STATE_READY)
    return this->state;

  osalDbgAssert(this->state == SENSOR_STATE_SLEEP, "Invalid state");

  /* start worker thread back */
  worker = chThdCreateStatic(Mpu6050ThreadWA, sizeof(Mpu6050ThreadWA),
                                    MPU6050PRIO, Mpu6050Thread, this);
  osalDbgAssert(nullptr != worker, "Can not allocate RAM");

  /* wakeup sensor */
  txbuf[0] = MPUREG_PWR_MGMT1;
  if (MSG_OK != transmit(txbuf, 1, &b, 1))
    goto ERROR;
  txbuf[0] = MPUREG_PWR_MGMT1;
  txbuf[1] = b & ~DEVICE_SLEEP;
  if (MSG_OK != transmit(txbuf, 2, NULL, 0))
    goto ERROR;

  this->state = SENSOR_STATE_READY;
  return this->state;

ERROR:
  this->state = SENSOR_STATE_DEAD;
  return this->state;
}

/**
 *
 */
void MPU6050::extiISR(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  osalSysLockFromISR();

  if (0 == isr_dlpf){ /* we need software rate divider */
    isr_count++;
    if (isr_count >= isr_smplrtdiv) {
      isr_count = 0;
      isr_sem.signalI();
    }
  }
  else { /* signal semaphore every interrupt pulse */
    isr_sem.signalI();
  }

  osalSysUnlockFromISR();
}

/**
 *
 */
float MPU6050::dT(void) {
  return smplrtdiv_current / static_cast<float>(1000);
}

