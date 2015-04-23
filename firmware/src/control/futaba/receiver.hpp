#ifndef FUTABA_RECEIVER_HPP_
#define FUTABA_RECEIVER_HPP_

#include "putinrange.hpp"
#include "tumbler.hpp"
#include "manual_switch_enum.hpp"

#define RECEIVER_STATUS_NO_ERRORS         0
#define RECEIVER_STATUS_CONN_LOST         ((uint32_t)1 << 31)

#define MAX_RC_CHANNELS                   8

namespace control {

/**
 *
 */
struct RecevierOutput {
  /**
   * @brief   Status register
   * @details Low 16 bits reserved for 16 channels error flags (1 == error)
   */
  uint32_t status = 0;

  /**
   * @brief   Channel values normalized -1..1
   */
  float ch[MAX_RC_CHANNELS];

  /**
   * @brief   Manual switch state
   */
  ManualSwitch man = ManualSwitch::manual;
};

/**
 *
 */
class Receiver {
public:
  virtual void start(const uint32_t *timeout) = 0;
  virtual void stop(void) = 0;
  virtual void update(RecevierOutput &result) = 0;
protected:
  float pwm_normalize(uint16_t v, float shift, float scale) const {
    return putinrange(((float)v - shift) / scale, -1, 1);
  }
  bool ready = false;
  const uint32_t *timeout = nullptr;
  Tumbler3<int, 1000, 1500, 2000, 150> manual_switch;
};

} /* namespace */

#endif /* FUTABA_RECEIVER_HPP_ */
