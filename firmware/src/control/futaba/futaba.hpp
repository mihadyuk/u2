#ifndef FUTABA_HPP_
#define FUTABA_HPP_

#include <futaba_data.hpp>
#include <futaba/tumbler.hpp>
#include <futaba/receiver_mavlink.hpp>
#include <futaba/receiver_pwm.hpp>

namespace control {

/**
 *
 */
class Futaba {
public:
  Futaba(void);
  void start(void);
  void stop(void);
  msg_t update(FutabaData &result);
private:
  bool ready = false;
  const uint32_t *timeout = nullptr;
  const uint32_t *override = nullptr;
  ReceiverPWM receiver_rc;
  ReceiverMavlink receiver_mavlink;
};

} /* namespace */

#endif /* FUTABA_HPP_ */
