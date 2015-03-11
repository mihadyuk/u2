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
  msg_t semiauto_interpret(receiver_data_t const &recv, FutabaData &result);
  msg_t man_switch_interpret(receiver_data_t const &recv, FutabaData &result);
  bool ready = false;
  const uint32_t *timeout = nullptr;
  const uint32_t *override = nullptr;
  ReceiverPWM receiver_rc;
  ReceiverMavlink receiver_mavlink;
};

} /* namespace */

#endif /* FUTABA_HPP_ */
