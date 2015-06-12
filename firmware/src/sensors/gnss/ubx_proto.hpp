#ifndef UBX_PROTO_HPP_
#define UBX_PROTO_HPP_

#include "string.h" // for memset

namespace gnss {

#define UBX_MAX_MSG_LEN           256
#define UBX_PAYLOAD_OFFSET        6U
#define UBX_OVERHEAD_TOTAL        8U

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
  CHECKSUM2,
  WAIT_HARVEST  // message collected and checksum is ok
};

/**
 * @brief   combined class ID and message ID in single enum
 * @note    class ID written in LSB for easier message packing using memcpy()
 */
enum class ubx_msg_t : uint16_t {
  EMPTY       = 0x0000,

  NAV_POSLLH  = 0x0201,
  NAV_VELNED  = 0x1201,
  NAV_TIMEUTC = 0x2101,

  CFG_MSG     = 0x0106,
  CFG_NAV5    = 0x2406,
  CFG_RATE    = 0x0806,

  ACK_ACK     = 0x0105,
  ACK_NACK    = 0x0005,
};

/**
 *
 */
struct ubx_ack_ack {
  ubx_msg_t msg_type = ubx_msg_t::EMPTY;
} __attribute__((packed));

/**
 *
 */
struct ubx_ack_nack {
  ubx_msg_t msg_type = ubx_msg_t::EMPTY;
} __attribute__((packed));

/**
 *
 */
struct ubx_cfg_rate {
  uint16_t measRate = 1000; /* milliseconds */
  uint16_t navRate = 1;     /* always 1 */
  uint16_t timeRef = 0;     /* 0: UTC time, 1: GPS time */
} __attribute__((packed));

/**
 *
 */
struct ubx_cfg_prt {
  ubx_cfg_prt(void) {memset(this, 0, sizeof(*this));}
  uint8_t  portID;
  uint8_t  reserved1;
  uint16_t txready;
  uint32_t mode;
  uint32_t baudrate;
  uint16_t inProtoMask;
  uint16_t outProtoMask;
  uint16_t flags;
  uint8_t  reserved2[2];
} __attribute__((packed));

/**
 *
 */
struct ubx_nav_posllh {
  uint32_t iTOW;// ms GPS time of week
  int32_t  lon; // deg 10e-7
  int32_t  lat; // deg 10e-7
  int32_t  h;   // mm Height above ellipsoid
  int32_t  hMSL;// mm
  uint32_t hAcc;// mm Horizontal accuracy estimate
  uint32_t vAcc;// mm Vertical accuracy estimate
} __attribute__((packed));

/**
 *
 */
struct ubx_nav_velned {
  uint32_t iTOW;  // ms GPS time of week
  int32_t  velN;  // cm/s
  int32_t  velE;  // cm/s
  int32_t  velD;  // cm/s
  uint32_t speed; // cm/s Speed module (3D)
  uint32_t gSpeed;// cm/s Speed module (2D)
  int32_t  hdg;   // deg * 1e-5
  uint32_t sAcc;  // cm/s Speed accuracy estimate
  uint32_t cAcc;  // deg Coarse/heading accuracy estimate
} __attribute__((packed));

/**
 *
 */
struct ubx_nav_timeutc {
  uint32_t iTOW;  // ms GPS time of week
  uint32_t tAcc;  // ns Time accuracy estimate
  int32_t  nano;  // ns Fraction of second -1e9..1e9
  uint16_t year;  // 1999..2099
  uint8_t  month; // 1..12
  uint8_t  day;   // 1..31
  uint8_t  hour;  // 0..23
  uint8_t  min;   // 0..59
  uint8_t  sec;   // 0..60
  uint8_t  valid; // Validity  flags
} __attribute__((packed));

//#error "USE NAV PVT message instead of all previouse"
//#error "CFG_NAV5 to set needed math model"
//#error "CFG_MSG to set output rate"
//#error "CFG_TP5 time pulse param"
//#error "CFG_PRT for protocols on port"

/**
 *
 */
class UbxBuf {
public:
  UbxBuf(void);
  void push(uint8_t b);
  void reset(void) {
    tip = 0;
  }
  size_t get_len(void) {
    return tip;
  }
  uint8_t data[UBX_MAX_MSG_LEN];
private:
  size_t tip;
};

/**
 *
 */
class UbxProto {
public:
  UbxProto(void);
  ubx_msg_t collect(uint8_t byte);
  template <typename T> size_t pack(const T &msg, ubx_msg_t type, uint8_t *buf, size_t buflen);
  template <typename T> void unpack(T &msg);
  void drop(void);
private:
  void checksum(const uint8_t *data, size_t len, uint8_t *result);
  bool checksum_ok(void);
  size_t pack_impl(uint8_t *buf, ubx_msg_t type, uint16_t N, const void *data);
  void reset(void);
  collect_state_t state = collect_state_t::START1;
  size_t current_len = 0;
  UbxBuf buf;
  size_t dbg_rx_bytes = 0;
  size_t dbg_message_bytes = 0;
  uint16_t dbg_overflow_cnt = 0;
  uint16_t dbg_unknown_msg_cnt = 0;
};

/**
 *
 */
template <typename T>
void UbxProto::unpack(T &msg) {
  memcpy(&msg, &this->buf.data[UBX_PAYLOAD_OFFSET], sizeof(msg));
  this->reset();
}

/**
 *
 */
template <typename T>
size_t UbxProto::pack(const T &msg, ubx_msg_t type, uint8_t *buf, size_t buflen) {
  uint16_t datalen = sizeof(msg);

  if (buflen < (UBX_OVERHEAD_TOTAL + datalen))
    return 0; // not enough room in buffer
  else
    return this->pack_impl(buf, type, datalen, &msg);
}

} /* namespace */

#endif /* UBX_PROTO_HPP_ */


