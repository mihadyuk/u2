#ifndef ADIS_HPP_
#define ADIS_HPP_

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

class Adis {
public:
  Adis(void);
  bool start(void);
  void stop(void);
  uint16_t get(adis_data_t *result);
  adisfp dt(void);
  static void extiISR(EXTDriver *extp, expchannel_t channel);
  msg_t wait(systime_t timeout);

private:
  bool ready;
  time_measurement_t tm;
};

extern Adis adis;

#endif /* ADIS_HPP_ */
