#ifndef ADIS_HPP_
#define ADIS_HPP_

typedef float adisfp; /* adis floating point type */

class Adis {
public:
  Adis(void);
  bool start(void);
  void stop(void);
  uint16_t get(adisfp *acc, adisfp *gyr, adisfp *mag,
               adisfp *baro, adisfp *quat, adisfp *euler);
  adisfp dt(void);
  static void extiISR(EXTDriver *extp, expchannel_t channel);
  msg_t wait(systime_t timeout);

private:
  bool ready;
  time_measurement_t tm;
};

extern Adis adis;

#endif /* ADIS_HPP_ */
