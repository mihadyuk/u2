#pragma GCC optimize "-O2"

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
static const uint8_t UBX_SYNC_1 = 0xB5;
static const uint8_t UBX_SYNC_2 = 0x62;

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
size_t UbxProto::pack_impl(uint8_t *buf, ubx_msg_t type,
                           uint16_t N, const void *data) {
  buf[0] = UBX_SYNC_1;
  buf[1] = UBX_SYNC_2;
  memcpy(&buf[2], &type, 2);
  memcpy(&buf[4], &N, 2);
  memcpy(&buf[UBX_PAYLOAD_OFFSET], data, N);
  checksum(&buf[2], 4+N, &buf[UBX_PAYLOAD_OFFSET+N]);

  return UBX_OVERHEAD_TOTAL + N;
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

/**
 *
 */
UbxBuf::UbxBuf(void) : tip(0) {
  memset(this->data, 0, sizeof(this->data));
}

void UbxBuf::push(uint8_t b) {
  data[tip] = b;
  tip++;
}

/**
 *
 */
bool UbxProto::checksum_ok(void) {
  uint8_t sum[2];
  size_t L = buf.get_len() - 4;
  this->checksum(&buf.data[2], L, sum);
  return 0 == memcmp(sum, &buf.data[L+2], 2);
}

/**
 *
 */
ubx_msg_t UbxProto::extract_rtti(uint8_t *data) {
  ubx_msg_t ret;
  memcpy(&ret, &data[2], sizeof(ret));
  return ret;
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
  return;
}

/**
 *
 */
void UbxProto::reset(void) {
  state = collect_state_t::START1;
  buf.reset();
}

/**
 *
 */
ubx_msg_t UbxProto::collect(uint8_t b) {
  ubx_msg_t ret = ubx_msg_t::EMPTY;
  this->dbg_rx_bytes++;

  switch (state) {
  case collect_state_t::START1:
    if (UBX_SYNC_1 == b) {
      buf.push(b);
      state = collect_state_t::START2;
    }
    else
      reset();
    break;

  /**/
  case collect_state_t::START2:
    if (UBX_SYNC_2 == b) {
      buf.push(b);
      state = collect_state_t::CLASS;
    }
    else
      reset();
    break;

  /**/
  case collect_state_t::CLASS:
    buf.push(b);
    state = collect_state_t::ID;
    break;

  /**/
  case collect_state_t::ID:
    buf.push(b);
    state = collect_state_t::LEN1;
    break;

  /**/
  case collect_state_t::LEN1:
    buf.push(b);
    current_len = b;
    state = collect_state_t::LEN2;
    break;

  /**/
  case collect_state_t::LEN2:
    buf.push(b);
    current_len |= b << 8;
    if (current_len > UBX_MSG_BUF_LEN - UBX_OVERHEAD_TOTAL) {
      this->dbg_overflow_cnt++;
      reset();
    }
    else {
      if (current_len > 0)
        state = collect_state_t::PAYLOAD;
      else
        state = collect_state_t::CHECKSUM1;
    }
    break;

  /**/
  case collect_state_t::PAYLOAD:
    if (current_len > 0) {
      buf.push(b);
      current_len--;
      if (0 == current_len)
        state = collect_state_t::CHECKSUM1;
    }
    break;

  /**/
  case collect_state_t::CHECKSUM1:
    buf.push(b);
    state = collect_state_t::CHECKSUM2;
    break;

  /**/
  case collect_state_t::CHECKSUM2:
    buf.push(b);
    if (checksum_ok()) {
      memcpy(&ret, &buf.data[2], 2);
      state = collect_state_t::WAIT_HARVEST;
    }
    else {
      this->dbg_bad_crc++;
      reset();
    }
    break;

  case collect_state_t::WAIT_HARVEST:
    /* You can not call parser until collected data pending in buffer.
       You have to call unpack() or dropMessage() method to correctly
       exit from this state */
    osalSysHalt("Error trap");
    break;
  }

  return ret;
}

/**
 *
 */
void UbxProto::drop(void) {

  osalDbgCheck(this->state == collect_state_t::WAIT_HARVEST);
  this->dbg_drop_msg++;
  this->reset();
}
