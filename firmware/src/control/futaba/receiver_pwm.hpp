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
  void update(receiver_data_t &result);
  friend void futaba_cb(EICUDriver *eicup, eicuchannel_t channel, uint32_t w, uint32_t p);
private:
  float get_ch(int32_t map) const;
  const int32_t *map_ail = nullptr; /* -1 denotes "unused" */
  const int32_t *map_ele = nullptr;
  const int32_t *map_rud = nullptr;
  const int32_t *map_thr = nullptr;
  const int32_t *map_man = nullptr;
};

} /* namespace */

#endif /* RECEIVER_PWM_HPP_ */
