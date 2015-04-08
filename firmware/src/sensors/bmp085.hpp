#ifndef BMP085_H_
#define BMP085_H_

#include "alpha_beta.hpp"
#include "i2c_sensor.hpp"

#define bmp085addr          0b1110111

#define BMP085_RX_DEPTH     24
#define BMP085_TX_DEPTH     4

/**
 *
 */
typedef enum {
  MEASURE_NONE = 0,   /* nothing measuring currently */
  MEASURE_T,          /* current measurement is temperature */
  MEASURE_P,          /* current measurement is pressure */
} measurement_t;

/**
 *
 */
typedef struct {
  uint32_t  p;              /* Pascals */
  uint32_t  p_msl_adjusted; /* Pascals */
  float     altitude;       /* m */
  float     climb;          /* m/s */
} abs_pressure_data_t;

/**
 *
 */
class BMP085: private I2CSensor {
public:
  BMP085(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(abs_pressure_data_t &result);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  sensor_state_t get_state(void) {return this->state;}
  void stop(void);
  void sleep(void);
  static void extiISR(EXTDriver *extp, expchannel_t channel);

private:
  bool hw_init_full(void);
  bool hw_init_fast(void);

  float climb(float alt);
  bool start_t_measurement(void);
  bool start_p_measurement(void);
  bool acquire_t(void);
  bool acquire_p(void);
  void picle(abs_pressure_data_t &result);
  void calc_pressure(void);
  uint8_t rxbuf[BMP085_RX_DEPTH];
  uint8_t txbuf[BMP085_TX_DEPTH];
  filters::AlphaBetaVariableLen<float> altitude_filter;
  filters::AlphaBetaVariableLen<float> climb_filter;
  int32_t const *flen_pres_stat, *flen_climb;
  int32_t const *above_msl;
  measurement_t measure_type;
  float altitude_prev;
  // uncompensated temperature and pressure values
  uint32_t up, ut;
  // bmp085 calibration coefficients
  int16_t  ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16_t ac4, ac5, ac6;
  abs_pressure_data_t cache;
  uint32_t pressure_compensated;
  thread_t *worker;
  friend void bmp085Thread(void *arg);
};

bool TrigCalibrateBaro();

#endif /* BMP085_H_ */
