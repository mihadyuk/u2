#ifndef FUTABA_PWM_HPP_
#define FUTABA_PWM_HPP_

#define FUTABA_PWM_CHANNELS         4

class FutabaPWM {
public:
  void start(void);
  void stop(void);
  void update(uint16_t *pwm);
  friend void futaba_cb(EICUDriver *eicup, eicuchannel_t channel);
private:
  bool ready = false;
  static uint16_t cache[FUTABA_PWM_CHANNELS];
};

#endif /* FUTABA_PWM_HPP_ */
