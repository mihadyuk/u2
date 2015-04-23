#ifndef FUTABA_HPP_
#define FUTABA_HPP_

#include <futaba_output.hpp>
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
  msg_t update(FutabaOutput &result, float dT);
private:
  msg_t semiauto_interpret(RecevierOutput const &recv, FutabaOutput &result);
  msg_t man_switch_interpret(RecevierOutput const &recv, FutabaOutput &result);
  float pwm_normalize(uint16_t v, float shift, float scale) const {
    return putinrange(((float)v - shift) / scale, -1, 1);
  }
  bool ready = false;
  const uint32_t *timeout  = nullptr;
  const uint32_t *override = nullptr;
  const int32_t  *map_man  = nullptr; /* -1 denotes "unused" */
  ReceiverPWM receiver_rc;
  ReceiverMavlink receiver_mavlink;
  Tumbler3<int, 1000, 1500, 2000, 150> manual_switch;
};

} /* namespace */

#endif /* FUTABA_HPP_ */
