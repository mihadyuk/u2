#ifndef RECEIVER_PWM_FPGA_HPP_
#define RECEIVER_PWM_FPGA_HPP_

#include "receiver.hpp"

namespace control {

/**
 *
 */
class ReceiverPWMFPGA : public Receiver {
public:
  void start(void);
  void stop(void);
  void update(RecevierOutput &result);
private:
  uint16_t get_ch(size_t chnum, bool *data_valid) const;
};

} /* namespace */

#endif /* RECEIVER_PWM_FPGA_HPP_ */
