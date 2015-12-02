#ifndef FUTABA_RECEIVER_HPP_
#define FUTABA_RECEIVER_HPP_

#include "array_len.hpp"

#define MAX_RC_CHANNELS                   8

namespace control {

/**
 *
 */
struct RecevierOutput {
  /**
   *
   */
  RecevierOutput(void) {
    for (size_t i=0; i<ArrayLen(ch); i++)
      ch[i] = 1500;
  }

  /**
   * @brief   Channel values (2000..1000). MSB denotes error.
   */
  uint16_t ch[MAX_RC_CHANNELS];

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

  /**
   * @brief   Any error raise this flag
   */
  bool data_valid = false;
};

/**
 *
 */
class Receiver {
public:
  virtual void start(void) = 0;
  virtual void stop(void) = 0;
  virtual void update(RecevierOutput &result) = 0;
protected:
  bool ready = false;
};

} /* namespace */

#endif /* FUTABA_RECEIVER_HPP_ */
