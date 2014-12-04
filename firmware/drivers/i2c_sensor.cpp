#include "main.h"
#include "i2c_sensor.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
I2CSensor::I2CSensor(I2CDriver *i2cdp, i2caddr_t addr):
  state(SENSOR_STATE_STOP),
  i2cdp(i2cdp),
  addr(addr)
{
  return;
}

/**
 *
 */
void I2CSensor::error_handler(msg_t status){

  if (MSG_OK == status)
    return;
  else {
    #if I2C_SENSOR_USE_ERROR_COUNTERS
    i2cflags_t flags = i2cGetErrors(this->i2cdp);
    if (flags & I2C_ACK_FAILURE)
      ack_failure++;
    if (flags & I2C_BUS_ERROR)
      bus_error++;
    if (flags & I2C_ARBITRATION_LOST)
      arbitration_lost++;
    if (flags & I2C_OVERRUN)
      overrun++;
    #endif
  }

  if (status != MSG_OK){
    i2cStop(i2cdp);
    chThdSleepMilliseconds(1);
    #if I2C_SENSOR_USE_ERROR_COUNTERS
    bus_restart++;
    #endif
    i2cStart(i2cdp, i2cdp->config);
  }
}

/**
 * transaction wrapper
 */
msg_t I2CSensor::transmit(const uint8_t *txbuf, size_t txbytes,
                                 uint8_t *rxbuf, size_t rxbytes){
  msg_t status = MSG_OK;

  i2cAcquireBus(this->i2cdp);
  status = i2cMasterTransmitTimeout(this->i2cdp, this->addr,
                      txbuf, txbytes, rxbuf, rxbytes, MS2ST(6));
  i2cReleaseBus(this->i2cdp);
  error_handler(status);

  return status;
}

/**
 * transaction wrapper
 */
msg_t I2CSensor::receive(uint8_t *rxbuf, size_t rxbytes){

  msg_t status = MSG_OK;

  i2cAcquireBus(this->i2cdp);
  status = i2cMasterReceiveTimeout(this->i2cdp, this->addr,
                                  rxbuf, rxbytes, MS2ST(6));
  i2cReleaseBus(this->i2cdp);
  error_handler(status);

  return status;
}

/**
 *
 */
sensor_state_t I2CSensor::get_state(void) {
  return state;
}
