#ifndef FUTABA_HPP_
#define FUTABA_HPP_

#include <control/futaba/tumbler.hpp>
#include <control/futaba_data.hpp>
#include <control/futaba/receiver_mavlink.hpp>
#include <control/futaba/receiver_synth.hpp>
#include <futaba/receiver_ppm.hpp>
#include <futaba/receiver_pwm.hpp>

namespace control {

/**
 * @brief   State of manual tumbler
 */
typedef enum {
  MANUAL_SWITCH_MANUAL    = 0,
  MANUAL_SWITCH_STABILIZE = 1,
  MANUAL_SWITCH_MISSION   = 2,
}manual_switch_t;

/**
 *
 */
class Futaba {
public:
  msg_t update(FutabaData &result);
  void start(void);
  void stop(void);
private:
  bool ready = false;
  const uint32_t *timeout = NULL;
  ReceiverMavlink receiver_mavlink;
  ReceiverPWM receiver_rc;
  ReceiverSynth receiver_synth;
  Tumbler3<int, 900, 1200, 1400, 1600, 1800, 2100> switch_mavlink;
  Tumbler3<int, 900, 1200, 1400, 1600, 1800, 2100> switch_rc;
  Tumbler3<int, 900, 1200, 1400, 1600, 1800, 2100> switch_synth;
};

} /* namespace */

#endif /* FUTABA_HPP_ */
