#ifndef MARG_HPP_
#define MARG_HPP_

#include "adis.hpp"
#include "mpu6050.hpp"
#include "ak8975.hpp"
#include "lsm303_acc.hpp"
#include "lsm303_mag.hpp"
#include "marg_data.hpp"

/**
 *
 */
class Marg {
public:
  Marg(Adis &adis);
  void stop(void);
  sensor_state_registry_t start(void);
  sensor_state_registry_t get_state(void);
  msg_t get(marg_data_t &result, systime_t timeout);
private:
  void sleep_all(void);
  sensor_state_registry_t reschedule(void);
  Adis &adis;
  MPU6050 mpu6050;
  AK8975 ak8975;
  LSM303_mag lsm303mag;
  LSM303_acc lsm303acc;
  bool ready = false;
  const uint32_t *gyr_src, *acc_src, *mag_src;
  uint8_t gyr_src_prev, acc_src_prev, mag_src_prev;
};

#endif /* MARG_HPP_ */
