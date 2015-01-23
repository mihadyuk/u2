#ifndef FUTABA_ROVER_HPP_
#define FUTABA_ROVER_HPP_

#include <control/futaba/pwm_receiver.hpp>
#include <control/futaba/tumbler.hpp>

#include "impact.hpp"

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
  Futaba(Impact &impact, PWMReceiver &pwm_receiver);
  bool update(manual_switch_t *manual, systime_t pwm_timeout);
  bool dryRun(manual_switch_t *manual, systime_t pwm_timeout);
  void failSafe(void);
  void start(void);
  void stop(void);
private:
  float pwm_normalize(uint16_t v);
  float pwm_normalize_thrust(uint16_t v);
  void pwm2impact(const PwmVector &pwm, Impact &impact);
  bool process_pwm(manual_switch_t *manual, systime_t pwm_timeout);
  Impact &impact;
  PWMReceiver &pwm_receiver;
  PwmVector pwm_vector;
  bool ready;
  Tumbler3<int, 900, 1200, 1400, 1600, 1800, 2100> manual_switch3;
};

#endif /* FUTABA_ROVER_HPP_ */
