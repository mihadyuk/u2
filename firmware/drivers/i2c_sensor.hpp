#ifndef I2C_SENSOR_HPP_
#define I2C_SENSOR_HPP_

#include "sensor.hpp"

#define I2C_SENSOR_USE_ERROR_COUNTERS     FALSE

class I2CSensor{
public:
  I2CSensor(I2CDriver *i2cdp, const i2caddr_t addr);

protected:
  sensor_state_t get_state(void);
  msg_t transmit(const uint8_t *txbuf, size_t txbytes,
                       uint8_t *rxbuf, size_t rxbytes);
  msg_t receive(uint8_t *rxbuf, size_t rxbytes);
  virtual void stop(void) = 0;
  virtual void sleep(void) = 0;
  virtual sensor_state_t start(void) = 0;
  virtual sensor_state_t wakeup(void) = 0;
  sensor_state_t state;

private:
  void error_handler(msg_t status);
  virtual bool hw_init_fast(void) = 0;
  virtual bool hw_init_full(void) = 0;

  I2CDriver *i2cdp;
  const i2caddr_t addr;
  #if I2C_SENSOR_USE_ERROR_COUNTERS
    uint16_t ack_failure;
    uint16_t bus_error;
    uint16_t arbitration_lost;
    uint16_t overrun;
    uint16_t bus_restart;
  #endif
};

#endif /* I2C_SENSOR_HPP_ */
