#ifndef PID_CHANNEL_ENUM_HPP_
#define PID_CHANNEL_ENUM_HPP_

namespace control {

typedef enum {
  PID_CHAIN_AIL = 0,
  PID_CHAIN_ELE = 1,
  PID_CHAIN_RUD = 2,
  PID_CHAIN_THR = 3,
  PID_CHAIN_ENUM_END
} pid_chain_t;

} // namespace

#endif /* PID_CHANNEL_ENUM_HPP_ */

