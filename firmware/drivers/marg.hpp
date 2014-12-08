#ifndef MARG_HPP_
#define MARG_HPP_

#include "adis.hpp"
#include "mpu6050.hpp"
#include "ak8975.hpp"
#include "lsm303_acc.hpp"
#include "lsm303_mag.hpp"

#define MARG_SENSOR_DATA_BITS   3

/**
 *
 */
typedef struct {
  uint32_t lsm303acc: MARG_SENSOR_DATA_BITS;
  uint32_t lsm303mag: MARG_SENSOR_DATA_BITS;
  uint32_t mpu6050:   MARG_SENSOR_DATA_BITS;
  uint32_t ak8975:    MARG_SENSOR_DATA_BITS;
  uint32_t adis:      MARG_SENSOR_DATA_BITS;
} marg_state_reg_t;

/**
 *
 */
typedef struct {
  float dt;
  float acc[3];
  float gyr[3];
  float mag[3];
  marg_state_reg_t reg;
} marg_data_t;

/**
 *
 */
class Marg {
public:
  Marg(void);
  void stop(void);
  marg_state_reg_t start(void);
  marg_state_reg_t get_state(void);
  msg_t update(marg_data_t *data, systime_t timeout);
private:
  void sleep_all(void);
  marg_state_reg_t reschedule(void);
  chibios_rt::BinarySemaphore adis_sem;
  chibios_rt::BinarySemaphore mpu6050_sem;
  Adis adis;
  MPU6050 mpu6050;
  AK8975 ak8975;
  LSM303_mag lsm303mag;
  LSM303_acc lsm303acc;
  bool ready = false;
  const uint32_t *gyr_src, *acc_src, *mag_src;
  uint8_t gyr_src_prev, acc_src_prev, mag_src_prev;
};

#endif /* MARG_HPP_ */
