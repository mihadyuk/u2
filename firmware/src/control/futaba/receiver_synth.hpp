#ifndef FUTABA_RECEIVER_SYNTH_HPP_
#define FUTABA_RECEIVER_SYNTH_HPP_

#include <futaba/receiver.hpp>

namespace control {

/**
 *
 */
class ReceiverSynth : public Receiver {
public:
  void start(const uint32_t *timeout);
  void stop(void);
  void update(receiver_data_t &result);
};

} /* namespace */

#endif /* FUTABA_RECEIVER_SYNTH_HPP_ */
