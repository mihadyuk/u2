#ifndef LSM303_MAG_HPP_
#define LSM303_MAG_HPP_

#include "i2c_sensor.hpp"
#include "marg_data.hpp"

#define lsm303magaddr       0b0011110

#define LSM_MAG_RX_DEPTH    8
#define LSM_MAG_TX_DEPTH    4

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

private:
  msg_t set_gain(uint8_t val);
  msg_t refresh_gain(void);
  msg_t start_single_measurement(void);
  msg_t get_prev_measurement(float *result, int16_t *result_raw);
  msg_t stop_sleep_code(void);
  void thermo_comp(float *result);
  void iron_comp(float *result);
  float mag_sens(void);
  void pickle(float *result, int16_t *result_raw);
  bool hw_init_full(void);
  bool hw_init_fast(void);
  uint8_t rxbuf[LSM_MAG_RX_DEPTH];
  uint8_t txbuf[LSM_MAG_TX_DEPTH];
  size_t sample_cnt;
  const uint32_t *gain = NULL;
  float cache[3];
  int16_t cache_raw[3];
  uint8_t gain_prev;
};

#endif /* LSM303_MAG_HPP_ */
