#include <ctime>
#include <cmath>
#include <cstring>
#include <cstdlib>

#include "main.h"
#include "ubx_proto.hpp"
#include "array_len.hpp"
#include "endianness.h"

#if BYTE_ORDER != LITTLE_ENDIAN
#error "For big endian machine packing and unpacking code must be rewritten"
#endif

using namespace gnss;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define PAYLOAD_OFFSET    6U
#define OVERHEAD_TOTAL    8U

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/**
 *
 */
size_t UbxProto::actual_pack(uint8_t *buf, ubx_msg_t type,
                             uint16_t N, const void *data) {
  buf[0] = 0xB5;
  buf[1] = 0x62;
  memcpy(&buf[2], &type, 2);
  memcpy(&buf[4], &N, 2);
  memcpy(&buf[PAYLOAD_OFFSET], data, N);
  checksum(&buf[2], 4+N, &buf[PAYLOAD_OFFSET+N]);

  return OVERHEAD_TOTAL + N;
}

/**
 *
 */
void UbxProto::checksum(const uint8_t *data, size_t len, uint8_t *result) {
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  for (size_t i=0; i<len; i++) {
    ck_a += data[i];
    ck_b += ck_a;
  }

  result[0] = ck_a;
  result[1] = ck_b;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
UbxProto::UbxProto(void) {
  memset(this->rxbuf, 0, sizeof(this->rxbuf));
}

/**
 *
 */
size_t UbxProto::pack(const ubx_cfg_rate &msg, uint8_t *buf, size_t buflen) {
  uint16_t datalen = sizeof(msg);

  if (buflen < (OVERHEAD_TOTAL + datalen))
    return 0; // not enough room in buffer
  else
    return actual_pack(buf, ubx_msg_t::CFG_RATE, datalen, &msg);
}

/**
 *
 */
ubx_msg_t UbxProto::collect(uint8_t byte) {
  (void)byte;
  this->dbg_rx_bytes++;
  osalSysHalt("unrealized");
  return ubx_msg_t::EMPTY;
}

/**
 *
 */
void UbxProto::unpack(ubx_cfg_rate &msg) {
  memcpy(&msg, &this->rxbuf[PAYLOAD_OFFSET], sizeof(msg));
}



