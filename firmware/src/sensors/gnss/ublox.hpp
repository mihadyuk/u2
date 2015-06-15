#ifndef UBLOX_HPP_
#define UBLOX_HPP_

#include "gnss_receiver.hpp"
#include "ubx_proto.hpp"

namespace gnss {

/**
 *
 */
enum class ubx_ack_t {
  NACK  = 0,
  ACK   = 1,
  NONE  = 2
};

/**
 *
 */
class uBlox : public GNSSReceiver {
public:
  uBlox(SerialDriver *sdp);
  void start(void);
private:
  static THD_FUNCTION(ubxRxThread, arg);
  void gnss2mavlink(const ubx_nav_pvt &pvt);
  void set_fix_period(uint16_t msec);
  void set_port(void);
  void set_dyn_model(uint32_t dyn_model);
  void set_message_rate(void);
  ubx_ack_t wait_ack(ubx_msg_t type, systime_t timeout);
  template <typename T> void write_with_confirm(T msg, ubx_msg_t type, systime_t timeout);
  void configure(uint32_t dyn_model, uint32_t fix_period);
  void update_settings(void);
  void pvtdispatch(const ubx_nav_pvt &pvt);
  const uint32_t *dyn_model = nullptr;
  const uint32_t *fix_period = nullptr;
  uint32_t dyn_model_cache = 8;
  uint32_t fix_period_cache = 200;
};

} // namespace

#endif /* UBLOX_HPP_ */
