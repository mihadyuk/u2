#ifndef LSM303_MAG_HPP_
#define LSM303_MAG_HPP_

#include "i2c_sensor.hpp"
#include "marg_data.hpp"

#define LSM303_MAG_I2C_ADDR   0b0011110

#define LSM_MAG_RX_DEPTH      8
#define LSM_MAG_TX_DEPTH      4

class LSM303_mag: private I2CSensor {
public:
  LSM303_mag(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(marg_data_t &result);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  sensor_state_t get_state(void) {return this->state;}
  void stop(void);
  void sleep(void);
  static void extiISR(EXTDriver *extp, expchannel_t channel);
  static void vtimerISR(void *p);

private:
  msg_t set_gain(uint8_t val);
  msg_t param_update(void);
  msg_t start_single_measurement(void);
  msg_t get_prev_measurement(marg_vector_t &result, marg_vector_raw_t &result_raw);
  msg_t stop_sleep_code(void);
  void enable_interrupts(void);
  void disable_interrupts(void);
  void thermo_comp(marg_vector_t &result);
  void iron_comp(marg_vector_t &result);
  float mag_sens(void);
  void pickle(marg_vector_t &result, marg_vector_raw_t &result_raw);
  bool hw_init_full(void);
  bool hw_init_fast(void);
  uint8_t rxbuf[LSM_MAG_RX_DEPTH];
  uint8_t txbuf[LSM_MAG_TX_DEPTH];
  size_t sample_cnt;
  const uint32_t *gain = NULL;
  marg_vector_t cache;
  marg_vector_raw_t cache_raw;
  uint8_t gain_current;
};

#endif /* LSM303_MAG_HPP_ */
