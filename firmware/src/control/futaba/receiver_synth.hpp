#ifndef FUTABA_RECEIVER_SYNTH_HPP_
#define FUTABA_RECEIVER_SYNTH_HPP_

#include <futaba/receiver.hpp>

namespace control {

/**
 *
 */
class ReceiverSynth : public Receiver {
public:
  ReceiverSynth(systime_t timeout);
  void start(void);
  void stop(void);
  msg_t update(uint16_t *pwm) const;
};

} /* namespace */

#endif /* FUTABA_RECEIVER_SYNTH_HPP_ */
