#ifndef ADIS_HPP_
#define ADIS_HPP_

class Adis {
public:
  Adis(void);
  bool start(void);
  void stop(void);
  uint16_t get(float *acc, float *gyr, float *mag,
               float *baro, float *quat, float *euler);
  float dt(void);
  static void extiISR(EXTDriver *extp, expchannel_t channel);
  msg_t wait(systime_t timeout);

private:
  bool ready;
  time_measurement_t tm;
};

extern Adis adis;

#endif /* ADIS_HPP_ */
