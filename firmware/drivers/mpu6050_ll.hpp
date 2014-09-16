#ifndef MPU6050_LL_H_
#define MPU6050_LL_H_

#include "stdint.h"
#include "i2c_sensor.hpp"
#include "fir.hpp"

#define mpu6050addr_1     0b1101001
#define mpu6050addr_2     0b1101000

/* buffers depth */
#define MPU_RX_DEPTH 15  /* 1 status bite and 14 bytes of data */
#define MPU_TX_DEPTH 4

#define MPU_ACCEL_OFFSET  1
#define MPU_TEMP_OFFSET   7
#define MPU_GYRO_OFFSET   9

#define MPU_SMPLRT_DIV    0x19
#define MPU_CONFIG        0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_INT_PIN_CFG   0x37
#define MPU_INT_ENABLE    0x38
#define MPU_INT_STATUS    0x3A /* 1 status bite and 14 bytes of data */
#define MPU_PWR_MGMT1     0x6B
#define MPU_PWR_MGMT2     0x6C

//#define MPU6050_ACC_FILTER_LEN  257
#define MPU6050_FILTER_LEN  257

class MPU6050_LL: protected I2CSensor{
public:
  MPU6050_LL(I2CDriver *i2cdp, i2caddr_t addr);
  msg_t update_gyr(int16_t *result);
  msg_t update_acc(int16_t *result);
  msg_t start(void);
  void stop(void);

private:
  #if MPU6050_1KHZ
  FIR<float, int16_t, MPU6050_FILTER_LEN> acc_fir[3];
  FIR<float, int16_t, MPU6050_FILTER_LEN> gyr_fir[3];
  #endif
  void pickle_gyr(int16_t *result);
  void pickle_acc(int16_t *result);
  msg_t hw_init_full(void);
  msg_t hw_init_fast(void);
  uint8_t mpu_rxbuf[MPU_RX_DEPTH];
  uint8_t mpu_txbuf[MPU_TX_DEPTH];
  bool hw_initialized;
};



#endif /* MPU6050_LL_H_ */
