#ifndef FUTABA_HPP_
#define FUTABA_HPP_

#include <futaba_output.hpp>
#include <futaba/tumbler.hpp>
#include <futaba/receiver_mavlink.hpp>
#include <futaba/receiver_pwm.hpp>

#include "alpha_beta.hpp"
#include "acs_input.hpp"

namespace control {

/**
 *
 */
class Futaba {
public:
  Futaba(void);
  void start(void);
  void stop(void);
  void update(ACSInput &result, float dT);
private:
  void process_man_tumbler(RecevierOutput const &recv, ManualSwitch &man);
  void recevier2futaba(RecevierOutput const &recv, ACSInput &result);

  ReceiverPWM receiver_rc;
  ReceiverMavlink receiver_mavlink;

  bool ready = false;
  const uint32_t *timeout  = nullptr;
  const uint32_t *override = nullptr;
  const int32_t  *map_man  = nullptr; /* -1 denotes "unused" */

  Tumbler3<int, 1000, 1500, 2000, 150> manual_switch;

  HysteresisBool<uint32_t, 10, 30, true> hyst;
  filters::AlphaBeta<int32_t, 100> error_rate;
};

} /* namespace */

#endif /* FUTABA_HPP_ */
