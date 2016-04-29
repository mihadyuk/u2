#ifndef BMP085_HPP_
#define BMP085_HPP_

#include "baro_data.hpp"
#include "alpha_beta.hpp"
#include "i2c_sensor.hpp"

#define BMP085_I2C_ADDR     0b1110111

#define BMP085_RX_DEPTH     24
#define BMP085_TX_DEPTH     4

/**
 *
 */
class BMP085: private I2CSensor {
public:
  BMP085(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(baro_abs_data_t &result);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  sensor_state_t get_state(void) {return this->state;}
  void stop(void);
  void sleep(void);
  static void extiISR(EXTDriver *extp, expchannel_t channel);

private:
  bool hw_init_full(void);
  bool hw_init_fast(void);
  bool start_t_measurement(void);
  bool start_p_measurement(void);
  bool acquire_t(void);
  bool acquire_p(void);
  void picle(baro_abs_data_t &result);
  void calc_pressure(void);
  uint8_t rxbuf[BMP085_RX_DEPTH] = {0};
  uint8_t txbuf[BMP085_TX_DEPTH] = {0};
  // uncompensated temperature and pressure values
  uint32_t up = 0, ut = 0;
  // bmp085 calibration coefficients
  int16_t  ac1 = 0, ac2 = 0, ac3 = 0, b1 = 0, b2 = 0, mb = 0, mc = 0, md = 0;
  uint16_t ac4 = 0, ac5 = 0, ac6 = 0;
  baro_abs_data_t cache;
  uint32_t pressure_compensated = 0;
  thread_t *worker = nullptr;
  friend void bmp085Thread(void *arg);
};

#endif /* BMP085_HPP_ */
