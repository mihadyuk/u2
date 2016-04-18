#ifndef ADIS_HPP_
#define ADIS_HPP_

#include "sensor.hpp"
#include "marg_data.hpp"
#include "ahrs_data.hpp"

/**
 *
 */
typedef struct {
  marg_vector_t acc;
  marg_vector_t gyr;
  marg_vector_t mag;
  float baro;
  std::array<float, 4> quat;
  marg_vector_t euler;
  float temp;
  uint16_t errors;
} adis_measurement_t;

/**
 *
 */
class Adis : public Sensor {
public:
  Adis(void);
  void stop(void);
  void sleep(void);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  msg_t waitData(systime_t timeout);
  sensor_state_t get(ahrs_data_t &result);
  sensor_state_t get(marg_data_t &result);
  static void extiISR(EXTDriver *extp, expchannel_t channel);

private:
  friend THD_FUNCTION(AdisThread, arg);
  void set_kalman(void);
  void set_zero_offset(void);
  void set_sample_rate(void);
  void param_update(void);
  void acquire_data(void);
  bool hw_init_fast(void);
  bool hw_init_full(void);
  void set_lock(void);
  void release_lock(void);
  float dT(void);

  time_measurement_t tm;
  chibios_rt::BinarySemaphore protect_sem;
  static chibios_rt::BinarySemaphore isr_sem;
  chibios_rt::BinarySemaphore data_ready_sem;
  thread_t *worker = nullptr;
  adis_measurement_t measurement;
  const uint32_t *smplrtdiv = nullptr;
  uint8_t smplrtdiv_current = 24;
};

#endif /* ADIS_HPP_ */
