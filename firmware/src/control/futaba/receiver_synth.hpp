#ifndef FUTABA_RECEIVER_SYNTH_HPP_
#define FUTABA_RECEIVER_SYNTH_HPP_

#include <futaba/receiver.hpp>

namespace control {

/**
 *
 */
class ReceiverSynth : public Receiver {
public:
  void start(void);
  void stop(void);
  void update(RecevierOutput &result);
};

} /* namespace */

#endif /* FUTABA_RECEIVER_SYNTH_HPP_ */
