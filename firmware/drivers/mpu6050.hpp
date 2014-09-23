#ifndef MPU6050_HPP_
#define MPU6050_HPP_

#include "i2c_sensor.hpp"
#include "fir.hpp"

#define mpu6050addr         0b1101000

#define MPU_RX_DEPTH        15  /* 1 status bite and 14 bytes of data */
#define MPU_TX_DEPTH        4

#define MPU6050_1KHZ        FALSE
#define MPU6050_FIR_LEN     65

class MPU6050: protected I2CSensor{
public:
  MPU6050(I2CDriver *i2cdp, i2caddr_t addr);
  msg_t get(float *acc, float *gyr);
  msg_t start(void);
  void stop(void);
  float update_perod(void);
private:
  msg_t set_gyr_fs(uint8_t fs);
  msg_t set_acc_fs(uint8_t fs);
  msg_t refresh_fs(void);
  float gyr_sens(void);
  float acc_sens(void);
  void gyro_thermo_comp(float *result);
  void acc_egg_comp(float *result);
  void pickle_gyr(float *result);
  void pickle_acc(float *result);
  void pickle_temp(float *result);
  msg_t hw_init_full(void);
  msg_t hw_init_fast(void);
  float temperature;
  const uint32_t *gyr_fs = NULL;
  const uint32_t *acc_fs = NULL;
  const int32_t  *fir_f = NULL;
  uint8_t gyr_fs_prev;
  uint8_t acc_fs_prev;
  bool hw_initialized;
  FIR<float, int16_t, MPU6050_FIR_LEN> acc_fir[3];
  FIR<float, int16_t, MPU6050_FIR_LEN> gyr_fir[3];
  uint8_t rxbuf[MPU_RX_DEPTH];
  uint8_t txbuf[MPU_TX_DEPTH];
};

#endif /* MPU6050_HPP_ */
