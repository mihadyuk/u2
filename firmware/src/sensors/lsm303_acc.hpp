#ifndef LSM303_ACC_HPP_
#define LSM303_ACC_HPP_

#include "i2c_sensor.hpp"
#include "marg_data.hpp"

#define LSM303_ACC_I2C_ADDR     0b0011001

#define LSM_ACC_RX_DEPTH        8
#define LSM_ACC_TX_DEPTH        8

class LSM303_acc: private I2CSensor {
public:
  LSM303_acc(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(marg_data_t &result);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  sensor_state_t get_state(void) {return this->state;}
  void stop(void);
  void sleep(void);

private:
  float acc_sens(void);
  void pickle(float *result, int16_t *raw_ret);
  msg_t stop_sleep_code(void);
  bool hw_init_full(void);
  bool hw_init_fast(void);
  uint8_t rxbuf[LSM_ACC_RX_DEPTH];
  uint8_t txbuf[LSM_ACC_TX_DEPTH];
};

#endif /* LSM303_ACC_HPP_ */
