#ifndef AK8975_HPP_
#define AK8975_HPP_

#include "i2c_sensor.hpp"

#define ak8975addr       0x0C

/* buffers depth */
#define AK_RX_DEPTH 14

class AK8975: protected I2CSensor {
public:
  AK8975(I2CDriver *i2cdp, i2caddr_t addr);
  msg_t get(float *mag);
  msg_t start(void);
  void stop(void);
private:
  void thermo_comp(float *result);
  void iron_comp(float *result);
  float mag_sens(void);
  void pickle(float *result);
  msg_t hw_init_full(void);
  msg_t hw_init_fast(void);
  msg_t start_measurement(void);
  uint8_t rxbuf[AK_RX_DEPTH];
  float cache[3];
};

#endif /* AK8975_HPP_ */
