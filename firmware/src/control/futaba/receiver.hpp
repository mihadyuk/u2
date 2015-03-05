#ifndef FUTABA_RECEIVER_HPP_
#define FUTABA_RECEIVER_HPP_

#define RECEIVER_MAX_CHANNELS         4

#define RECEIVER_MIN_WIDTH            1000
#define RECEIVER_NEUTRAL_WIDTH        1500
#define RECEIVER_MAX_WIDTH            2000

#define RECEIVER_STATUS_CONN_LOST     ((uint32_t)1 << 31)

namespace control {

/**
 *
 */
struct receiver_data_t {
  /**
   * @brief   Status register
   * @details Low 16 bits reserved for 16 channels error flags (1 == error)
   */
  uint32_t status = RECEIVER_STATUS_CONN_LOST | 0xFFFF;
  /**
   * @brief   Channel values in uS.
   */
  uint16_t pwm[RECEIVER_MAX_CHANNELS] = {1500};
};

/**
 *
 */
class Receiver {
public:
  virtual void start(systime_t timeout) = 0;
  virtual void stop(void) = 0;
  virtual void update(receiver_data_t &result) const = 0;
protected:
  bool ready = false;
  systime_t timeout = S2ST(5);
};

} /* namespace */

#endif /* FUTABA_RECEIVER_HPP_ */
