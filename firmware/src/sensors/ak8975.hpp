#ifndef AK8975_HPP_
#define AK8975_HPP_

#include "i2c_sensor.hpp"
#include "marg_data.hpp"

#define AK8975_I2C_ADDR       0x0C

/* buffers depth */
#define AK8975_RX_DEPTH       14

class AK8975: protected I2CSensor {
public:
  AK8975(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(marg_data_t &result);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  sensor_state_t get_state(void) {return this->state;}
  void stop(void);
  void sleep(void);
private:
  void thermo_comp(marg_vector_t &result);
  void iron_comp(marg_vector_t &result);
  float mag_sens(void);
  void pickle(marg_vector_t &result, marg_vector_raw_t &result_raw);
  bool hw_init_full(void);
  bool hw_init_fast(void);
  msg_t start_measurement(void);
  uint8_t rxbuf[AK8975_RX_DEPTH];
  marg_vector_t cache;
  marg_vector_raw_t cache_raw;
};

#endif /* AK8975_HPP_ */
