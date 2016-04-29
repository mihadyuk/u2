#ifndef NPA700_HPP_
#define NPA700_HPP_

#include "i2c_sensor.hpp"

#define NPA700_I2C_ADDR     0b00101000

/* buffers depth */
#define NPA700_RX_DEPTH     4


class NPA700: private I2CSensor {
public:
  NPA700(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(baro_diff_data_t &result);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  bool ping(void);
  void stop(void);
  void sleep(void);

private:
  void pickle(baro_diff_data_t &result);
  bool hw_init_full(void);
  bool hw_init_fast(void);
  uint32_t const *mid = nullptr;
  float const *sens = nullptr;
  uint8_t rxbuf[NPA700_RX_DEPTH] = {0};
};

#endif /* NPA700_HPP_ */
