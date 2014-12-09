#ifndef AK8975_HPP_
#define AK8975_HPP_

#include "i2c_sensor.hpp"

#define ak8975addr       0x0C

/* buffers depth */
#define AK_RX_DEPTH 14

class AK8975: protected I2CSensor {
public:
  AK8975(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(float *result, int16_t *result_raw);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  sensor_state_t get_state(void) {return this->state;}
  void stop(void);
  void sleep(void);
private:
  void thermo_comp(float *result);
  void iron_comp(float *result);
  float mag_sens(void);
  void pickle(float *result, int16_t *result_raw);
  bool hw_init_full(void);
  bool hw_init_fast(void);
  msg_t start_measurement(void);
  uint8_t rxbuf[AK_RX_DEPTH];
  float cache[3];
  int16_t cache_raw[3];
};

#endif /* AK8975_HPP_ */
