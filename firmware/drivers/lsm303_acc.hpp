#ifndef LSM303_ACC_LL_H_
#define LSM303_ACC_LL_H_

#include "i2c_sensor.hpp"

#define lsm303accaddr       0b0011001

#define LSM_ACC_RX_DEPTH    8
#define LSM_ACC_TX_DEPTH    8

#define CTRL_REG1_A         0x20
#define CTRL_REG2_A         0x21
#define OUT_X_L_A           0x28

class LSM303_acc_LL: private I2CSensor{
public:
  LSM303_acc_LL(I2CDriver *i2cdp, i2caddr_t addr);
  msg_t update(int16_t *result);
  msg_t start(void);
  void stop(void);

private:
  void pickle(int16_t *result);
  msg_t hw_init_full(void);
  msg_t hw_init_fast(void);
  uint8_t rxbuf[LSM_ACC_RX_DEPTH];
  uint8_t txbuf[LSM_ACC_TX_DEPTH];
};

#endif /* LSM303_ACC_LL_H_ */
