#ifndef MPU6050_HPP_
#define MPU6050_HPP_

#include "i2c_sensor.hpp"
#include "fir.hpp"
#include "iir.hpp"
#include "marg_data.hpp"

#define MPU6050_I2C_ADDR    0b1101000

#define MPU_RX_DEPTH        16  /* 1 status byte + 14 bytes of data + 1 padding */
#define MPU_TX_DEPTH        4

#define MPU6050_FIR_LEN     257

#define IIR_LEN     2
#define IIR_SEC     2

#define POLYC_LEN           3   /* thermal compensation polynomial order + 1 */

/**
 *
 */
template <typename T>
struct MPU6050_fir_block {
  MPU6050_fir_block(const std::array<float, MPU6050_FIR_LEN> &taps) {
    for (size_t i=0; i<3; i++) {
      acc[i].setKernel(taps);
      gyr[i].setKernel(taps);
    }
  }
  MPU6050_fir_block(void) = delete;

  filters::FIR<T, MPU6050_FIR_LEN> acc[3];
  filters::FIR<T, MPU6050_FIR_LEN> gyr[3];
};

/**
 *
 */
//template <typename T>
//struct MPU6050_iir_block {
//  MPU6050_iir_block(const T *taps_a, const T *taps_b) {
//    for (size_t i=0; i<3; i++) {
//      acc[i].setKernel(taps_a, taps_b);
//      gyr[i].setKernel(taps_a, taps_b);
//    }
//  }
//  MPU6050_iir_block(void) = delete;
//
//  filters::IIR<T, MPU6050_IIR_LEN> acc[3];
//  filters::IIR<T, MPU6050_IIR_LEN> gyr[3];
//};


/**
 *
 */
template <typename T>
struct MPU6050_iir_block {
  MPU6050_iir_block(const T **taps_a, const T **taps_b, const T *gain) {
    for (size_t i=0; i<3; i++) {
      acc[i].setKernel(taps_a, taps_b, gain);
      gyr[i].setKernel(taps_a, taps_b, gain);
    }
  }
  MPU6050_iir_block(void) = delete;

  filters::IIRChain<T, IIR_LEN, IIR_SEC> acc[3];
  filters::IIRChain<T, IIR_LEN, IIR_SEC> gyr[3];
};




/**
 *
 */
class MPU6050: protected I2CSensor {
public:
  MPU6050(I2CDriver *i2cdp, i2caddr_t addr);
  msg_t waitData(systime_t timeout);
  sensor_state_t get(marg_data_t &result);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  sensor_state_t get_state(void) {return this->state;}
  void stop(void);
  void sleep(void);
  static void extiISR(EXTDriver *extp, expchannel_t channel);

private:
  friend THD_FUNCTION(Mpu6050Thread, arg);
  float dT(void);
  void acquire_data(void);
  msg_t soft_reset(void);
  msg_t acquire_simple(marg_vector_t &acc, marg_vector_t &gyr);
  msg_t acquire_fifo(marg_vector_t &acc, marg_vector_t &gyr);
  void set_lock(void);
  void release_lock(void);
  msg_t set_gyr_fs(uint8_t fs);
  msg_t set_acc_fs(uint8_t fs);
  msg_t set_dlpf_smplrt(uint8_t lpf, uint8_t smplrt);
  msg_t param_update(void);
  float gyr_sens(void);
  float acc_sens(void);
  void pickle_gyr(marg_vector_t &result);
  void pickle_acc(marg_vector_t &result);
  void pickle_fifo(marg_vector_t &acc, marg_vector_t &gyr, const size_t sample_cnt);
  bool hw_init_full(void);
  bool hw_init_fast(void);

  float temperature = 0;
  chibios_rt::BinarySemaphore protect_sem;
  chibios_rt::BinarySemaphore data_ready_sem;
  thread_t *worker = nullptr;
  marg_vector_t acc_data;
  marg_vector_t gyr_data;
  marg_vector_raw_t acc_raw_data;
  marg_vector_raw_t gyr_raw_data;

  const uint32_t *gyr_fs = nullptr;
  const uint32_t *acc_fs = nullptr;
  const uint32_t *dlpf = nullptr;
  const uint32_t *smplrtdiv = nullptr;
  const int32_t  *fir_f = nullptr;
  uint8_t gyr_fs_current = 0;
  uint8_t acc_fs_current = 0;
  uint8_t dlpf_current = 0;
  uint8_t smplrtdiv_current = 0;
  MPU6050_fir_block<float> &fir;
  MPU6050_iir_block<float> &iir;
  const uint32_t *MPUG_Tcomp_en = nullptr;
  const uint32_t *MPUA_Tcomp_en = nullptr;
  const float *gyr_bias_c[3*POLYC_LEN] = {};
  const float *acc_bias_c[3*POLYC_LEN] = {};
  const float *acc_sens_c[3*POLYC_LEN] = {};

  uint16_t fifo_remainder = 0;
  int16_t rxbuf_fifo[960 / 2] = {0};
  uint8_t rxbuf[MPU_RX_DEPTH] = {0};
  uint8_t txbuf[MPU_TX_DEPTH] = {0};

  static size_t  isr_count;
  static uint8_t isr_dlpf;
  static uint8_t isr_smplrtdiv;
  static chibios_rt::BinarySemaphore isr_sem;
};

#endif /* MPU6050_HPP_ */
