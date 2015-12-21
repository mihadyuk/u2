#ifndef PROTO_UBX_HPP_
#define PROTO_UBX_HPP_

#include "string.h" // for memset

namespace gnss {

#define UBX_MSG_BUF_LEN           512U
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
  NAV_DOP     = 0x0401,
  NAV_PVT     = 0x0701,
  NAV_VELNED  = 0x1201,
  NAV_TIMEUTC = 0x2101,
  NAV_SAT     = 0x3501,

  RXM_SFRBX   = 0x1302,
  RXM_RAWX    = 0x1502,

  ACK_NACK    = 0x0005,
  ACK_ACK     = 0x0105,

  CFG_PRT     = 0x0006,
  CFG_MSG     = 0x0106,
  CFG_RATE    = 0x0806,
  CFG_NAV5    = 0x2406,

  MON_VER     = 0x040A,
};

/**
 *    ACK_ACK
 */
struct ubx_ack_ack_payload {
  ubx_msg_t acked_msg = ubx_msg_t::EMPTY;
} __attribute__((packed));

struct ubx_ack_ack {
  ubx_ack_ack_payload payload;
  const ubx_msg_t rtti = ubx_msg_t::ACK_ACK;
  uint16_t recvd_bytes;
};

/**
 *    ACK_NACK
 */
struct ubx_ack_nack_payload {
  ubx_msg_t nacked_msg = ubx_msg_t::EMPTY;
} __attribute__((packed));

struct ubx_ack_nack {
  ubx_ack_nack_payload payload;
  const ubx_msg_t rtti = ubx_msg_t::ACK_NACK;
  uint16_t recvd_bytes;
};

/**
 *    CFG_RATE
 */
struct ubx_cfg_rate_payload {
  uint16_t measRate = 1000; /* milliseconds */
  uint16_t navRate = 1;     /* always 1 */
  uint16_t timeRef = 0;     /* 0: UTC time, 1: GPS time */
} __attribute__((packed));

struct ubx_cfg_rate {
  ubx_cfg_rate_payload payload;
  const ubx_msg_t rtti = ubx_msg_t::CFG_RATE;
  uint16_t recvd_bytes;
};

/**
 *    CFG_MSG
 */
struct ubx_cfg_msg_payload {
  ubx_msg_t msg_wanted = ubx_msg_t::EMPTY;
  uint8_t rate = 0;
} __attribute__((packed));

struct ubx_cfg_msg {
  ubx_cfg_msg_payload payload;
  const ubx_msg_t rtti = ubx_msg_t::CFG_MSG;
  uint16_t recvd_bytes;
};

/**
 *    CFG_PRT
 */
struct ubx_cfg_prt_payload {
  ubx_cfg_prt_payload(void) {memset(this, 0, sizeof(*this));}
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

struct ubx_cfg_prt {
  ubx_cfg_prt_payload payload;
  const ubx_msg_t rtti = ubx_msg_t::CFG_PRT;
  uint16_t recvd_bytes;
};

/**
 *    CFG_NAV5
 */
struct ubx_cfg_nav5_payload {
  ubx_cfg_nav5_payload(void) {memset(this, 0, sizeof(*this));}
  uint16_t mask;
  uint8_t  dynModel;
  uint8_t  fixMode;
  int32_t  fixedAlt;    // m
  uint32_t fixedAltVar; // m^2
  int8_t   minElev;     // deg
  uint8_t  drLimit;     // s Reserved
  uint16_t pDOP;        // 0.1
  uint16_t tDOP;        // 0.1
  uint16_t pAcc;        // m
  uint16_t tAcc;        // m
  uint8_t  holdThr;     // cm/s
  uint8_t  dgpsTimeout; // s
  uint8_t  cnoThrNumSVs;
  uint8_t  cnoThr;      // dbHz
  uint8_t  reserved1[2];
  uint16_t holdMaxDist; // m
  uint8_t  utcStandard;
  uint8_t  reserved2[5];
} __attribute__((packed));

struct ubx_cfg_nav5 {
  ubx_cfg_nav5_payload payload;
  const ubx_msg_t rtti = ubx_msg_t::CFG_NAV5;
  uint16_t recvd_bytes;
};

/**
 *    NAV_PVT
 */
struct ubx_nav_pvt_payload {
  ubx_nav_pvt_payload(void) {memset(this, 0, sizeof(*this));}
  uint32_t iTOW;    // ms GPS time of week
  uint16_t year;    // 1999..2099
  uint8_t  month;   // 1..12
  uint8_t  day;     // 1..31
  uint8_t  hour;    // 0..23
  uint8_t  min;     // 0..59
  uint8_t  sec;     // 0..60
  uint8_t  valid;   // Validity  flags
  uint32_t tAcc;    // ns Time accuracy estimate
  int32_t  nano;    // ns Fraction of second -1e9..1e9

  uint8_t  fixType;
  uint8_t  fixFlags;
  uint8_t  reserved;
  uint8_t  numSV;   // number of satellites used in solution

  int32_t  lon;     // deg 10e-7
  int32_t  lat;     // deg 10e-7
  int32_t  h;       // mm Height above ellipsoid
  int32_t  hMSL;    // mm
  uint32_t hAcc;    // mm Horizontal accuracy estimate
  uint32_t vAcc;    // mm Vertical accuracy estimate

  int32_t  velN;    // mm/s
  int32_t  velE;    // mm/s
  int32_t  velD;    // mm/s
  uint32_t gSpeed;  // mm/s Speed module (2D)
  int32_t  hdg;     // deg * 1e-5 Heading of motion
  uint32_t speedAcc;// mm/s Speed accuracy estimate
  uint32_t hdgAcc;  // deg * 1e-5 Coarse/heading accuracy estimate

  uint16_t pDOP;    // 0.01
  uint8_t  reserved2[6];
  int32_t  hedVeh;  // deg * 1e-5 Heading of vehicle
  uint8_t  reserved3[4];
} __attribute__((packed));

struct ubx_nav_pvt {
  ubx_nav_pvt_payload payload;
  const ubx_msg_t rtti = ubx_msg_t::NAV_PVT;
  uint16_t recvd_bytes;
};

/**
 *    NAV_SAT
 */
struct _svs_t {
  uint8_t  gnssID;
  uint8_t  svID;
  uint8_t  cno;   // dBHz. Carrier to noise ratio.
  int8_t   elev;  // deg. Elevation -90..+90. Unknown if out of range.
  int16_t  azim;  // deg. Azimuth -180..+180. Unknown if out of range.
  int16_t  prRes; // m/10. Pseudo range residual.
  uint32_t flags;
} __attribute__((packed));

template <size_t svs_cnt>
struct ubx_nav_sat_payload {
  ubx_nav_sat_payload(void) {memset(this, 0, sizeof(*this));}
  uint32_t iTOW;    // ms GPS time of week
  uint8_t  version;
  uint8_t  numSvs;
  uint8_t  reserved[2];
  _svs_t    svs[svs_cnt];
} __attribute__((packed));

template <size_t svs_cnt>
struct ubx_nav_sat {
  ubx_nav_sat_payload<svs_cnt> payload;
  const ubx_msg_t rtti = ubx_msg_t::NAV_SAT;
  uint16_t recvd_bytes;
};

/**
 *    NAV_DOP
 */
struct ubx_nav_dop_payload {
  ubx_nav_dop_payload(void) {memset(this, 0, sizeof(*this));}
  uint32_t iTOW;
  uint16_t gDOP;
  uint16_t pDOP;
  uint16_t tDOP;
  uint16_t vDOP;
  uint16_t hDOP;
  uint16_t nDOP;
  uint16_t eDOP;
} __attribute__((packed));

struct ubx_nav_dop {
  ubx_nav_dop_payload payload;
  const ubx_msg_t rtti = ubx_msg_t::NAV_DOP;
  uint16_t recvd_bytes;
};

/**
 *    MON_VER
 */
template <size_t ext_cnt>
struct ubx_mon_ver_payload {
  ubx_mon_ver_payload(void) {memset(this, 0, sizeof(*this));}
  char swVer[30];
  char hwVer[10];
  /* Extended optional fields. Set first index to desired value.
   * For example: NEO-M8N has 5 extended fields */
  char extended[ext_cnt][30];
} __attribute__((packed));

template <size_t ext_cnt>
struct ubx_mon_ver {
  ubx_mon_ver_payload<ext_cnt> payload;
  const ubx_msg_t rtti = ubx_msg_t::MON_VER;
  uint16_t recvd_bytes;
};

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
  size_t get_len(void) const {
    return tip;
  }
  uint8_t data[UBX_MSG_BUF_LEN];
private:
  size_t tip;
};

/**
 *
 */
class ProtoUbx {
public:
  ProtoUbx(void);
  ubx_msg_t collect(uint8_t byte);
  size_t packPollRequest(ubx_msg_t type, uint8_t *buf, size_t buflen) const;
  template <typename T> size_t pack(const T &msg, uint8_t *buf, size_t buflen);
  template <typename T> void unpack(T &result);
  void drop(void);
private:
  ubx_msg_t extract_rtti(uint8_t *data) const;
  uint16_t extract_len(uint8_t *data) const;
  void checksum(const uint8_t *data, size_t len, uint8_t *result) const;
  bool checksum_ok(void) const;
  size_t pack_impl(uint8_t *buf, ubx_msg_t type, uint16_t N, const void *data) const;
  void reset(void);
  collect_state_t state = collect_state_t::START1;
  size_t current_len = 0;
  UbxBuf buf;
  size_t dbg_rx_bytes = 0;
  size_t dbg_message_bytes = 0;
  uint16_t dbg_bad_crc = 0;
  uint16_t dbg_drop_msg = 0;
  uint16_t dbg_overflow_cnt = 0;
};

/**
 *
 */
template <typename T>
void ProtoUbx::unpack(T &result) {
  size_t N;
  uint16_t recvd = extract_len(this->buf.data);

  osalDbgCheck(result.rtti == extract_rtti(this->buf.data));
  osalDbgCheck(this->state == collect_state_t::WAIT_HARVEST);

  /* correctly handles overflow in variable length messages */
  if (sizeof(result.payload) >= recvd)
    N = recvd;
  else
    N = sizeof(result.payload);
  memcpy(&result.payload, &this->buf.data[UBX_PAYLOAD_OFFSET], N);
  result.recvd_bytes = recvd;

  this->dbg_message_bytes += N + UBX_OVERHEAD_TOTAL;
  this->reset();
}

/**
 *
 */
template <typename T>
size_t ProtoUbx::pack(const T &msg, uint8_t *buf, size_t buflen) {
  uint16_t datalen = sizeof(msg.payload);

  if (buflen < (UBX_OVERHEAD_TOTAL + datalen))
    return 0; // not enough room in buffer
  else
    return this->pack_impl(buf, msg.rtti, datalen, &msg.payload);
}

} /* namespace */

#endif /* PROTO_UBX_HPP_ */
