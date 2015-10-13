#ifndef MS5806_HPP_
#define MS5806_HPP_

#include "i2c_sensor.hpp"
#include "baro_data.hpp"

#define ms5806addr        0b1110111

/* buffers depth */
#define MS5806_RX_DEPTH   4
#define MS5806_TX_DEPTH   4

#define MS5806_CAL_WORDS  8

class MS5806: private I2CSensor {
public:
  MS5806(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(baro_data_t &result);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  void stop(void);
  void sleep(void);

private:
  void pickle(baro_data_t &result);
  void calc_pressure(void);
  bool start_t_measurement(void);
  bool start_p_measurement(void);
  bool acquire_data(uint8_t *rxbuf);
  bool hw_init_full(void);
  bool hw_init_fast(void);
  baro_data_t cache;
  uint8_t rxbuf_t[MS5806_RX_DEPTH];
  uint8_t rxbuf_p[MS5806_RX_DEPTH];
  uint8_t txbuf[MS5806_TX_DEPTH];
  uint16_t C[MS5806_CAL_WORDS];
  thread_t *worker;
  friend void ms5806Thread(void *arg);
};

#endif /* MS5806_HPP_ */
