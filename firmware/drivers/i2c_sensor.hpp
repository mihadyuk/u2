#ifndef I2C_SENSOR_H_
#define I2C_SENSOR_H_

class I2CSensor{
public:
  I2CSensor(I2CDriver *i2cdp, const i2caddr_t addr);

protected:
  msg_t transmit(const uint8_t *txbuf, size_t txbytes,
                       uint8_t *rxbuf, size_t rxbytes);
  msg_t receive(uint8_t *rxbuf, size_t rxbytes);
  virtual msg_t start(void) = 0;
  virtual void stop(void) = 0;
  virtual msg_t hw_init_fast(void) = 0;
  virtual msg_t hw_init_full(void) = 0;
  bool ready;

private:
  void error_handler(msg_t status);

  I2CDriver *i2cdp;
  const i2caddr_t addr;
  uint8_t ack_failure;
  uint8_t bus_error;
  uint8_t arbitration_lost;
  uint8_t overrun;
  uint8_t bus_restart;
};

#endif /* I2C_SENSOR_H_ */
