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
 * @brief     Calculates requred timeout.
 */
systime_t I2CSensor::calc_timeout(size_t txbytes, size_t rxbytes) {
  const uint32_t bitsinbyte = 10;
  uint32_t tmo;
  tmo = ((txbytes + rxbytes + 1) * bitsinbyte * 1000);
  tmo /= this->i2cdp->config->clock_speed;
  tmo += 2; /* some additional milliseconds to be safer */
  return MS2ST(tmo);
}

/**
 *
 */
I2CSensor::I2CSensor(I2CDriver *i2cdp, i2caddr_t addr):
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
              txbuf, txbytes, rxbuf, rxbytes, calc_timeout(txbytes, rxbytes));
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
                          rxbuf, rxbytes, calc_timeout(0, rxbytes));
  i2cReleaseBus(this->i2cdp);
  error_handler(status);

  return status;
}

