#ifndef MPU6050_HPP_
#define MPU6050_HPP_

#include "i2c_sensor.hpp"
#include "fir.hpp"

#define mpu6050addr         0b1101000

#define MPU_RX_DEPTH        16  /* 1 status byte + 14 bytes of data + 1 padding */
#define MPU_TX_DEPTH        4

#define MPU6050_1KHZ        FALSE
#define MPU6050_FIR_LEN     65

class MPU6050: protected I2CSensor{
public:
  MPU6050(I2CDriver *i2cdp, i2caddr_t addr,
      chibios_rt::BinarySemaphore &data_ready_sem);
  sensor_state_t get(float *acc, float *gyr, int16_t *acc_raw, int16_t *gyr_raw);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  sensor_state_t get_state(void) {return this->state;}
  void stop(void);
  void sleep(void);
  float dt(void);
  static void extiISR(EXTDriver *extp, expchannel_t channel);

private:
  friend THD_FUNCTION(Mpu6050Thread, arg);
  void acquire_data(void);
  msg_t acquire_simple(float *acc, float *gyr);
  msg_t acquire_fifo(float *acc, float *gyr);
  void set_lock(void);
  void release_lock(void);
  msg_t set_gyr_fs(uint8_t fs);
  msg_t set_acc_fs(uint8_t fs);
  msg_t set_dlpf_smplrt(uint8_t lpf, uint8_t smplrt);
  msg_t refresh_settings(void);
  float gyr_sens(void);
  float acc_sens(void);
  void gyro_thermo_comp(float *result);
  void acc_egg_comp(float *result);
  void pickle_gyr(float *result);
  void pickle_fifo(float *acc, float *gyr, const size_t sample_cnt);
  void pickle_acc(float *result);
  void pickle_temp(float *result);
  bool hw_init_full(void);
  bool hw_init_fast(void);

  float temperature;
  chibios_rt::BinarySemaphore protect_sem;
  chibios_rt::BinarySemaphore &data_ready_sem;
  thread_t *worker;
  float acc_data[3];
  float gyr_data[3];
  int16_t acc_raw_data[3];
  int16_t gyr_raw_data[3];

  const uint32_t *gyr_fs = NULL;
  const uint32_t *acc_fs = NULL;
  const uint32_t *dlpf = NULL;
  const uint32_t *smplrt_div = NULL;
  const int32_t  *fir_f = NULL;
  uint8_t gyr_fs_prev;
  uint8_t acc_fs_prev;
  uint8_t dlpf_prev;
  uint8_t smplrt_prev;
  FIR<float, float, MPU6050_FIR_LEN> *acc_fir;
  FIR<float, float, MPU6050_FIR_LEN> *gyr_fir;

  uint16_t fifo_remainder = 0;
  int16_t rxbuf_fifo[960 / 2];
  uint8_t rxbuf[MPU_RX_DEPTH];
  uint8_t txbuf[MPU_TX_DEPTH];

  static size_t  isr_count;
  static uint8_t isr_dlpf;
  static uint8_t isr_smplrt_div;
  static chibios_rt::BinarySemaphore isr_sem;
};

#endif /* MPU6050_HPP_ */
