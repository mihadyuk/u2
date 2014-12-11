#include "main.h"

#include "exti_local.hpp"
#include "mpu6050.hpp"
#include "pack_unpack.h"
#include "geometry.hpp"
#include "param_registry.hpp"

#include "mpu6050_fir_taps.h"
#include "array_len.hpp"
#include "putinrange.hpp"

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
  #define FIFO_RST            (1 << 2)
#define MPUREG_PWR_MGMT1        0x6B
  #define DEVICE_RESET          (1 << 7)
  #define DEVICE_SLEEP          (1 << 6)
#define MPUREG_PWR_MGMT2        0x6C
#define MPUREG_FIFO_CNT         0x72 /* MSB. Next byte is LSB */
#define MPUREG_FIFO_DATA        0x74
#define MPUREG_WHO_AM_I         0x75
  #define WHO_AM_I_VAL          0X68

/**
 * @brief   Gyro full scale in deg/s
 */
typedef enum {
  MPU_GYRO_FULL_SCALE_250 = 0,
  MPU_GYRO_FULL_SCALE_500,
  MPU_GYRO_FULL_SCALE_1000,
  MPU_GYRO_FULL_SCALE_2000
} gyro_sens_t;

/**
 * @brief   Accel full scale in g
 */
typedef enum {
  MPU_ACC_FULL_SCALE_2 = 0,
  MPU_ACC_FULL_SCALE_4,
  MPU_ACC_FULL_SCALE_8,
  MPU_ACC_FULL_SCALE_16
} acc_sens_t;

/* reset fifo if it contains such amount of bytes */
#define FIFO_RESET_THRESHOLD  1024

/* how many bytes in single fifo sample */
#define BYTES_IN_SAMPLE       12

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
    (2 * 9.81)  / 32768.0,
    (4 * 9.81)  / 32768.0,
    (8 * 9.81)  / 32768.0,
    (16 * 9.81) / 32768.0
};

FIR<float, float, MPU6050_FIR_LEN> acc_fir_array[3] __attribute__((section(".ccm")));
FIR<float, float, MPU6050_FIR_LEN> gyr_fir_array[3] __attribute__((section(".ccm")));

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
float MPU6050::gyr_sens(void){
  return gyro_sens_array[*gyr_fs];
}

/**
 *
 */
float MPU6050::acc_sens(void){
  return acc_sens_array[*acc_fs];
}

/**
 *
 */
void MPU6050::pickle_temp(float *result){
  uint8_t *b = &rxbuf[MPU_TEMP_OFFSET];
  result[0] = static_cast<int16_t>(pack8to16be(b));
  result[0] /= 340;
  result[0] += 36.53f;
}

/**
 *
 */
void MPU6050::gyro_thermo_comp(float *result){
  (void)result;
}

/**
 *
 */
void MPU6050::acc_egg_comp(float *result){
  (void)result;
}

/**
 *
 */
static void toggle_endiannes16(uint8_t *data, const size_t len){

  osalDbgCheck(0 == (len % 2));
  uint8_t tmp;

  for (size_t i=0; i<len; i+=2){
    tmp = data[i];
    data[i] = data[i+1];
    data[i+1] = tmp;
  }
}

/**
 *
 */
void MPU6050::pickle_gyr(float *result) {

  int16_t raw[3];
  uint8_t *b = &rxbuf[MPU_GYRO_OFFSET];
  float sens = this->gyr_sens();

  toggle_endiannes16(b, 6);
  memcpy(raw, b, sizeof(raw));

  for (size_t i=0; i<3; i++) {
    gyr_raw_data[i] = raw[i];
    result[i] = sens * raw[i];
  }

  gyro_thermo_comp(result);
}

/**
 *
 */
void MPU6050::pickle_acc(float *result){

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

  acc_egg_comp(result);
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

  i2c_status = set_gyr_fs(*gyr_fs);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;
  osalThreadSleepMilliseconds(1);

  i2c_status = set_acc_fs(*acc_fs);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;
  osalThreadSleepMilliseconds(1);

  txbuf[0] = MPUREG_INT_PIN_CFG;
  txbuf[1] = INT_RD_CLEAR | I2C_BYPASS_EN;
  i2c_status = transmit(txbuf, 2, NULL, 0);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;
  osalThreadSleepMilliseconds(1);

  i2c_status = set_dlpf_smplrt(*this->dlpf, *this->smplrtdiv);
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
msg_t MPU6050::refresh_settings(void) {

  uint8_t fs, lpf, smplrt;

  msg_t ret1 = MSG_OK;
  msg_t ret2 = MSG_OK;
  msg_t ret3 = MSG_OK;

  /* gyr full scale */
  fs = *gyr_fs;
  if (fs != gyr_fs_prev){
    ret1 = set_gyr_fs(fs);
    gyr_fs_prev = fs;
  }

  /* acc full scale */
  fs = *acc_fs;
  if (fs != acc_fs_prev){
    ret2 = set_acc_fs(fs);
    acc_fs_prev = fs;
  }

  /* low pass filter and sample rate */
  lpf = *dlpf;
  smplrt = *smplrtdiv;
  if ((lpf != dlpf_prev) || (smplrt != smplrt_prev)){
    ret3 = set_dlpf_smplrt(lpf, smplrt);
    dlpf_prev = lpf;
    smplrt_prev = smplrt;
  }

  if ((MSG_OK == ret1) && (MSG_OK == ret2) && (MSG_OK == ret3))
    return MSG_OK;
  else
    return MSG_RESET;
}

/**
 *
 */
msg_t MPU6050::acquire_simple(float *acc, float *gyr) {

  msg_t ret = MSG_RESET;

  txbuf[0] = MPUREG_INT_STATUS;
  ret = transmit(txbuf, 1, rxbuf, sizeof(rxbuf));

  this->set_lock();

  pickle_temp(&temperature);
  if (nullptr != gyr)
    pickle_gyr(gyr);
  if (nullptr != acc)
    pickle_acc(acc);
  fifo_remainder = 0;

  this->release_lock();

  return ret;
}

/**
 *
 */
static time_measurement_t fir_tmu;

void MPU6050::pickle_fifo(float *acc, float *gyr, const size_t sample_cnt) {

  float sens;
  const size_t acc_fifo_offset = 0;
  const size_t gyr_fifo_offset = 3;

  for (size_t i=0; i<3; i++) {
    acc_raw_data[i] = rxbuf_fifo[acc_fifo_offset + i];
    gyr_raw_data[i] = rxbuf_fifo[gyr_fifo_offset + i];
  }

  if (sample_cnt == 10)
    chTMStartMeasurementX(&fir_tmu);
  for (size_t n=0; n<sample_cnt; n++) {
    for (size_t i=0; i<3; i++){
      size_t shift = n * BYTES_IN_SAMPLE / 2 + i;
      acc[i] = acc_fir[i].update(rxbuf_fifo[shift + acc_fifo_offset]);
      gyr[i] = gyr_fir[i].update(rxbuf_fifo[shift + gyr_fifo_offset]);
    }
  }
  if (sample_cnt == 10)
    chTMStopMeasurementX(&fir_tmu);

  /* acc */
  sens = this->acc_sens();
  acc[0] *= sens;
  acc[1] *= sens;
  acc[2] *= sens;
  acc_egg_comp(acc);

  /* gyr */
  sens = this->gyr_sens();
  gyr[0] *= sens;
  gyr[1] *= sens;
  gyr[2] *= sens;
  gyro_thermo_comp(gyr);
}

/**
 *
 */
msg_t MPU6050::acquire_fifo(float *acc, float *gyr) {

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

    txbuf[0] = MPUREG_FIFO_DATA;
    ret = transmit(txbuf, 1, (uint8_t*)rxbuf_fifo, recvd);
    toggle_endiannes16((uint8_t*)rxbuf_fifo, recvd);

    this->set_lock();
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
    if (*dlpf > 0)
      ret1 = acquire_simple(acc_data, gyr_data);
    else
      ret1 = acquire_fifo(acc_data, gyr_data);
    ret2 = refresh_settings();

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
static THD_WORKING_AREA(Mpu6050ThreadWA, 256);
THD_FUNCTION(Mpu6050Thread, arg) {
  chRegSetThreadName("Mpu6050");
  MPU6050 *self = static_cast<MPU6050 *>(arg);

  while (!chThdShouldTerminateX()) {
    self->isr_sem.wait();
    self->isr_dlpf = *self->dlpf;
    self->isr_smplrtdiv = *self->smplrtdiv;
    self->acquire_data();
    self->data_ready_sem.signal();
  }

  chThdExit(MSG_OK);
  return MSG_OK;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
MPU6050::MPU6050(I2CDriver *i2cdp, i2caddr_t addr,
                      chibios_rt::BinarySemaphore &data_ready_sem) :
I2CSensor(i2cdp, addr),
protect_sem(false),
data_ready_sem(data_ready_sem),
acc_fir(acc_fir_array),
gyr_fir(gyr_fir_array)
{
  state = SENSOR_STATE_STOP;
  chTMObjectInit(&fir_tmu);
}

/**
 *
 */
sensor_state_t MPU6050::start(void) {

  if (SENSOR_STATE_STOP == this->state) {
    acc_fir[0].setKernel(taps, ArrayLen(taps));
    acc_fir[1].setKernel(taps, ArrayLen(taps));
    acc_fir[2].setKernel(taps, ArrayLen(taps));

    gyr_fir[0].setKernel(taps, ArrayLen(taps));
    gyr_fir[1].setKernel(taps, ArrayLen(taps));
    gyr_fir[2].setKernel(taps, ArrayLen(taps));

    param_registry.valueSearch("MPU_gyr_fs",    &gyr_fs);
    param_registry.valueSearch("MPU_acc_fs",    &acc_fs);
    param_registry.valueSearch("MPU_fir_f",     &fir_f);
    param_registry.valueSearch("MPU_dlpf",      &dlpf);
    param_registry.valueSearch("MPU_smplrtdiv", &smplrtdiv);

    gyr_fs_prev = *gyr_fs;
    acc_fs_prev = *acc_fs;
    dlpf_prev   = *dlpf;
    smplrt_prev = *smplrtdiv;

    this->isr_dlpf = *dlpf;
    this->isr_smplrtdiv = *smplrtdiv;

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

  chThdTerminate(worker);
  isr_sem.reset(false); /* speedup termination */
  chThdWait(worker);
  worker = nullptr;

  Exti.mpu6050(false);

  txbuf[0] = MPUREG_PWR_MGMT1;
  txbuf[1] = DEVICE_RESET; /* soft reset */
  if (MSG_OK != transmit(txbuf, 2, NULL, 0))
    this->state = SENSOR_STATE_DEAD;
  else
    this->state = SENSOR_STATE_STOP;

  mpu6050_power_off();
}

/**
 *
 */
sensor_state_t MPU6050::get(float *acc, float *gyr,
                            int16_t *acc_raw, int16_t *gyr_raw) {

  if (SENSOR_STATE_READY == this->state) {
    set_lock();
    if (nullptr != acc)
      memcpy(acc, this->acc_data, sizeof(this->acc_data));
    if (nullptr != gyr)
      memcpy(gyr, this->gyr_data, sizeof(this->gyr_data));
    if (nullptr != acc_raw)
      memcpy(acc_raw, this->acc_raw_data, sizeof(this->acc_raw_data));
    if (nullptr != gyr_raw)
      memcpy(gyr_raw, this->gyr_raw_data, sizeof(this->gyr_raw_data));
    release_lock();
  }

  return this->state;
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
  osalThreadSleepMilliseconds(1);

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
float MPU6050::dt(void) {
  return *smplrtdiv / static_cast<float>(1000);
}

