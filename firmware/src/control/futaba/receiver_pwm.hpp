#ifndef RECEIVER_PWM_HPP_
#define RECEIVER_PWM_HPP_

#include "receiver.hpp"

namespace control {

/**
 *
 */
class ReceiverPWM : public Receiver {
public:
  void start(const uint32_t *timeout);
  void stop(void);
  void update(RecevierOutput &result);
  friend void futaba_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p);
private:
  void get_ch(size_t chnum, float *result, uint32_t *status) const;
  void get_tumbler(size_t chnum, ManualSwitch *result, uint32_t *status);
  const int32_t *map_man = nullptr;/* -1 denotes "unused" */
};

} /* namespace */

#endif /* RECEIVER_PWM_HPP_ */
