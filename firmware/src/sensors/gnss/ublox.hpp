#ifndef UBLOX_HPP_
#define UBLOX_HPP_

#include "gnss_receiver.hpp"
#include "proto_ubx.hpp"

namespace gnss {

/**
 *
 */
enum class ublox_ack_t {
  NACK  = 0,
  ACK   = 1,
  NONE  = 2
};

/**
 *
 */
class uBlox : public GNSSReceiver {
public:
  uBlox(SerialDriver *sdp, uint32_t start_baudrate,
                           uint32_t working_baudrate);
protected:
  void start_impl(void);
private:
  ProtoUbx parser;
  static THD_FUNCTION(ubxRxThread, arg);
  void get_version(void);
  void set_fix_period(uint16_t msec);
  void set_port(void);
  void set_dyn_model(uint32_t dyn_model);
  void set_message_rate(void);
  ubx_msg_t wait_any_timeout(const ubx_msg_t *type_list,
                            size_t list_len, systime_t timeout);
  ubx_msg_t wait_one_timeout(ubx_msg_t type, systime_t timeout);
  ublox_ack_t wait_ack(ubx_msg_t type, systime_t timeout);
  bool device_alive(systime_t timeout);
  template <typename T> void write_with_confirm(const T &msg, systime_t timeout);
  void configure(uint32_t dyn_model, uint32_t fix_period);
  void update_settings(void);
  void pvt_dop2mavlink(void);
  void sat2mavlink(void);
  void gnss_dispatch(void);
  const uint32_t *dyn_model = nullptr;
  const uint32_t *fix_period = nullptr;
  uint32_t dyn_model_cache = 8;
  uint32_t fix_period_cache = 200;
  uint16_t nack_cnt = 0;
  uint16_t no_answer_cnt = 0;
  ubx_nav_pvt pvt;
  ubx_nav_dop dop;
  ubx_mon_ver<5> ver;
  ubx_nav_sat<32> sat;
};

} // namespace

#endif /* UBLOX_HPP_ */
