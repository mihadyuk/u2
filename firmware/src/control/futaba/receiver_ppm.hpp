#ifndef RECEIVER_PPM_HPP_
#define RECEIVER_PPM_HPP_

#include <futaba/receiver.hpp>

namespace control {

/**
 *
 */
class ReceiverPPM : public Receiver {
public:
  void start(void);
  void stop(void);
  void update(RecevierOutput &result);
};

} /* namespace */

#endif /* RECEIVER_PPM_HPP_ */
