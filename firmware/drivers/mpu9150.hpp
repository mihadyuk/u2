#ifndef MPU9150_HPP_
#define MPU9150_HPP_

#include "mpu6050.hpp"
#include "ak8975.hpp"

class MPU9150 {
public:
  MPU9150(I2CDriver *i2cdp);
  msg_t start(void);
  void stop(void);
  msg_t get(float *acc, float *gyr, float *mag);
  float update_perod(void);
private:
  AK8975 ak;
  MPU6050 mpu;
  bool ready = false;
};

#endif /* MPU9150_HPP_ */
