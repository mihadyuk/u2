#ifndef ENGINE_2CH_HPP_
#define ENGINE_2CH_HPP_

#include <drivetrain_impact.hpp>
#include "drivetrain/drivetrain_pwm.hpp"
#include "futaba_data.hpp"

namespace control {

class Engine2ch {
public:
  Engine2ch(PWM &pwm);
  void start(void);
  void stop(void);
  void update(const FutabaData &futaba_data, const Impact &impact);
private:
  PWM &pwm;
  bool ready = false;
  const uint32_t *thr_min = nullptr, *thr_mid = nullptr, *thr_max = nullptr;
};

} /* namespace */

#endif /* ENGINE_2CH_HPP_ */
