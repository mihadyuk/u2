#ifndef AHRS_HPP_
#define AHRS_HPP_

#include "marg.hpp"
#include "ahrs_data.hpp"

/**
 *
 */
typedef enum {
  AHRS_STATE_UNINIT = 0,
  AHRS_STATE_STOP,
  AHRS_STATE_READY,
  AHRS_STATE_CALIBRATE,
  AHRS_STATE_UPDATE,
}ahrs_state_t;

/**
 *
 */
class Ahrs {
public:
  Ahrs(void);
  void start(void);
  void stop(void);
  msg_t get(ahrs_data_t &result, systime_t timeout);
private:
  msg_t get_adis(ahrs_data_t &result, systime_t timeout);
  msg_t get_starlino(ahrs_data_t &result, systime_t timeout);
  msg_t get_madgwick(ahrs_data_t &result, systime_t timeout);
  msg_t get_kalman(ahrs_data_t &result, systime_t timeout);
  void reschedule(void);
  Adis adis;
  Marg marg;
  ahrs_state_t state;
  const uint32_t *mode_;
  uint8_t mode_prev;
};

#endif /* AHRS_HPP_ */
