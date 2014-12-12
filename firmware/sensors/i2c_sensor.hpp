#ifndef I2C_SENSOR_HPP_
#define I2C_SENSOR_HPP_

#include "sensor.hpp"

#define I2C_SENSOR_USE_ERROR_COUNTERS     TRUE

class I2CSensor : public Sensor {
public:
  I2CSensor(I2CDriver *i2cdp, const i2caddr_t addr);

protected:
  msg_t transmit(const uint8_t *txbuf, size_t txbytes,
                       uint8_t *rxbuf, size_t rxbytes);
  msg_t receive(uint8_t *rxbuf, size_t rxbytes);

private:
  systime_t calc_timeout(size_t txbytes, size_t rxbytes);
  void error_handler(msg_t status);
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
