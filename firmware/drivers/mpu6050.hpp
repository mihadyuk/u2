#ifndef MPU6050_HPP_
#define MPU6050_HPP_

#include "i2c_sensor.hpp"

#define mpu6050addr       0b1101000

/* buffers depth */
#define MPU_RX_DEPTH 15  /* 1 status bite and 14 bytes of data */
#define MPU_TX_DEPTH 4

class MPU6050: protected I2CSensor{
public:
  MPU6050(I2CDriver *i2cdp, i2caddr_t addr);
  msg_t get(float *acc, float *gyr);
  msg_t start(void);
  void stop(void);
private:
  void pickle_gyr(float *result);
  void pickle_acc(float *result);
  msg_t hw_init_full(void);
  msg_t hw_init_fast(void);
  void set_dlpf(void);
  uint8_t mpu_rxbuf[MPU_RX_DEPTH];
  uint8_t mpu_txbuf[MPU_TX_DEPTH];
  bool hw_initialized;
};

#endif /* MPU6050_HPP_ */
