#ifndef FUTABA_RECEIVER_HPP_
#define FUTABA_RECEIVER_HPP_

#include "string.h"

#include "putinrange.hpp"
#include "tumbler.hpp"
#include "manual_switch_enum.hpp"

#define RECEIVER_STATUS_NO_ERRORS         0
#define RECEIVER_STATUS_CONN_LOST         ((uint16_t)1 << 15)
#define RECEIVER_STATUS_INCORRECT_VALUE   ((uint16_t)1 << 14)
#define RECEIVER_DATA_MASK                0xFFF
#define RECEIVER_FLAGS_MASK               0xF000

#define RECEIVER_MAX_VALID_VALUE          2000
#define RECEIVER_MIN_VALID_VALUE          1000

#define MAX_RC_CHANNELS                   8

namespace control {

/**
 *
 */
struct RecevierOutput {
  RecevierOutput(void) {
    memset(ch, 0xFF, sizeof(ch));
  }

  /**
   * @brief   Channel values (2000..1000). MSB contain flags
   */
  size_t ch[MAX_RC_CHANNELS];
  /**
   * @brief   Total number of channels available in this receiver
   * @details Set to 0 if no channels available
   */
  size_t channels = 0;
  /**
   * @brief   Numbers used in normalization procedure.
   * @details May differ from receiver to receiver.
   */
  uint16_t normalize_shift = 1500;
  uint16_t normalize_scale = 500;
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
  bool ready = false;
  const uint32_t *timeout = nullptr;
};

} /* namespace */

#endif /* FUTABA_RECEIVER_HPP_ */
