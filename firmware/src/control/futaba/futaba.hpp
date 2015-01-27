#ifndef FUTABA_HPP_
#define FUTABA_HPP_

#include <control/futaba/tumbler.hpp>
#include <control/futaba_data.hpp>
#include <control/futaba/receiver_mavlink.hpp>
#include <control/futaba/receiver_rc.hpp>
#include <control/futaba/receiver_synth.hpp>

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
  Futaba() = delete;
  Futaba(systime_t timeout);
  msg_t update(FutabaData &result);
  void start(void);
  void stop(void);
private:
  bool ready = false;
  const systime_t timeout;
  ReceiverMavlink receiver_mavlink;
  ReceiverRC receiver_rc;
  Tumbler3<int, 900, 1200, 1400, 1600, 1800, 2100> switch_mavlink;
  Tumbler3<int, 900, 1200, 1400, 1600, 1800, 2100> switch_rc;
};

} /* namespace */

#endif /* FUTABA_HPP_ */
