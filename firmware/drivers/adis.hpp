#ifndef ADIS_HPP_
#define ADIS_HPP_

#include "sensor.hpp"

typedef float adisfp; /* adis floating point type */

typedef struct {
  adisfp acc[3];
  adisfp gyr[3];
  adisfp mag[3];
  adisfp baro;
  adisfp quat[4];
  adisfp euler[3];
  adisfp temp;
  uint16_t errors;
} adis_data_t;

class Adis : public Sensor {
public:
  Adis(chibios_rt::BinarySemaphore &data_ready_sem);
  void stop(void);
  void sleep(void);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  sensor_state_t get(adis_data_t *result);
  adisfp dt(void);
  static void extiISR(EXTDriver *extp, expchannel_t channel);
  msg_t wait(systime_t timeout);

private:
  friend THD_FUNCTION(AdisThread, arg);
  void acquire_data(void);
  bool hw_init_fast(void);
  bool hw_init_full(void);

  time_measurement_t tm;
  static chibios_rt::BinarySemaphore interrupt_sem;
  chibios_rt::BinarySemaphore &data_ready_sem;
  thread_t *worker;
  adis_data_t measurement;
};

#endif /* ADIS_HPP_ */
