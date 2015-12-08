#ifndef FUTABA_HPP_
#define FUTABA_HPP_

#include <futaba/tumbler.hpp>
#include <futaba/receiver_mavlink.hpp>
#include <futaba/receiver_pwm.hpp>
#include <futaba/receiver_pwm_fpga.hpp>

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

#if defined(BOARD_BEZVODIATEL)
  ReceiverPWM receiver_rc;
#elif defined(BOARD_MNU)
  ReceiverPWMFPGA receiver_rc;
#else
#error "unknown board"
#endif
  ReceiverMavlink receiver_mavlink;

  bool ready = false;
  const uint32_t *timeout  = nullptr;
  const int32_t  *map_man  = nullptr; /* -1 denotes "unused" */

  Tumbler3<int, 900, 1500, 2100, 100> manual_switch;

  HysteresisBool<uint32_t, 10, 30, true> hyst;
  filters::AlphaBeta<int32_t, 25> error_rate;
  filters::AlphaBeta<int32_t, 10> manual_switch_filter;
};

} /* namespace */

#endif /* FUTABA_HPP_ */
