#ifndef RECEIVER_PWM_HPP_
#define RECEIVER_PWM_HPP_

#include "receiver.hpp"

namespace control {

/**
 *
 */
class ReceiverPWM : public Receiver {
public:
  void start(void);
  void stop(void);
  void update(RecevierOutput &result);
  //friend void futaba_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p);
private:
  uint16_t get_ch(size_t chnum, bool *data_valid) const;
};

} /* namespace */

#endif /* RECEIVER_PWM_HPP_ */
