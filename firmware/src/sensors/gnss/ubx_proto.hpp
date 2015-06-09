#ifndef UBX_PROTO_HPP_
#define UBX_PROTO_HPP_

#include "string.h" // for memset

namespace gnss {

#define UBX_MAX_MSG_LEN     256

/**
 *
 */
enum class collect_state_t {
  START1,     /* wait 0xB5 */
  START2,     /* wait 0x62 */
  CLASS,      /* message class */
  ID,         /* message id inside class */
  LEN1,
  LEN2,
  PAYLOAD,
  CHECKSUM1,
  CHECKSUM2
};

/**
 *
 */
enum class ubx_msg_t : uint16_t {
  CFG_MSG   = 0x0601,
  CFG_NAV5  = 0x0624,
  CFG_RATE  = 0x0608,
  UNKNOWN,
  RXBUF_OVERFLOW,
};

/**
 *
 */
struct ubx_cfg_rate {
  uint16_t measRate = 1000; /* milliseconds */
  uint16_t navRate = 1;     /* always 1 */
  uint16_t timeRef = 0;     /* 0: UTC time, 1: GPS time */
}__attribute__((packed));

/**
 *
 */
class UbxProto {
public:
  UbxProto(void);
  ubx_msg_t collect(uint8_t byte);
  size_t pack(const ubx_cfg_rate &msg, uint8_t *buf, size_t buflen);
  void unpack(ubx_cfg_rate &msg);
private:
  void checksum(const uint8_t *data, size_t len, uint8_t *result);
  size_t actual_pack(uint8_t *buf, ubx_msg_t type, uint16_t N, const void *data);
  void reset_collector(void);
  collect_state_t state = collect_state_t::START1;
  uint8_t rxbuf[UBX_MAX_MSG_LEN];
};

} /* namespace */

#endif /* UBX_PROTO_HPP_ */


