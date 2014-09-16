#ifndef LSM303_MAG_LL_H_
#define LSM303_MAG_LL_H_

#include "i2c_sensor.hpp"
#include "mems_mag_interface.hpp"

#define lsm303magaddr       0b0011110

#define LSM_MAG_RX_DEPTH    8
#define LSM_MAG_TX_DEPTH    4

#define CRA_REG_M           0x00
#define CRB_REG_M           0x01
#define MR_REG_M            0x02

#define OUT_X_H_M           0x03
#define TEMP_OUT_H_M        0x31


class LSM303_mag_LL: private MagInterface, private I2CSensor{
public:
  LSM303_mag_LL(I2CDriver *i2cdp, i2caddr_t addr);
  msg_t update(int16_t *result);
  msg_t start(void);
  void stop(void);

private:
  void pickle(int16_t *result);
  msg_t hw_init_full(void);
  msg_t hw_init_fast(void);
  uint8_t rxbuf[LSM_MAG_RX_DEPTH];
  uint8_t txbuf[LSM_MAG_TX_DEPTH];
  uint32_t sample_cnt;
};


#endif /* LSM303_MAG_LL_H_ */
