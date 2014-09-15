#ifndef PACK_UNPACK_H_
#define PACK_UNPACK_H_

/*
 * fe == foreign endian
 * ne == native endian
 * be == big endian
 * le == little endian
 */

#include "string.h"
#include "endianness.h"

#if BYTE_ORDER == LITTLE_ENDIAN

/******************************
 * Foreign endian
 *****************************/
static inline uint16_t pack8to16fe(const uint8_t *buf){
  uint16_t result = 0;
  result |= buf[0] << 8;
  result |= buf[1];
  return result;
}

static inline uint32_t pack8to32fe(const uint8_t *buf){
  uint32_t result = 0;
  result |= buf[0] << 24;
  result |= buf[1] << 16;
  result |= buf[2] << 8;
  result |= buf[3];
  return result;
}

static inline uint64_t pack8to64fe(const uint8_t *buf){
  uint64_t result = pack8to32fe(buf);
  result <<= 32;
  result |= pack8to32fe(buf + 4);
  return result;
}

static inline size_t unpack16to8fe(uint16_t v, uint8_t *buf){
  buf[0] = (v >> 8)  & 0xFF;
  buf[1] = (v >> 0)  & 0xFF;
  return sizeof(v);
}

static inline size_t unpack32to8fe(uint32_t v, uint8_t *buf){
  buf[0] = (v >> 24) & 0xFF;
  buf[1] = (v >> 16) & 0xFF;
  buf[2] = (v >> 8)  & 0xFF;
  buf[3] = (v >> 0)  & 0xFF;
  return sizeof(v);
}

static inline size_t unpack64to8fe(uint64_t v, uint8_t *buf){
  uint32_t msb = (v >> 32) & 0xFFFFFFFF;
  uint32_t lsb = v & 0xFFFFFFFF;
  unpack32to8fe(msb, buf);
  unpack32to8fe(lsb, buf + 4);
  return sizeof(v);
}

/******************************
 * Native endian
 *****************************/
static inline uint16_t pack8to16ne(const uint8_t *buf){
  uint16_t result;
  memcpy(&result, buf, sizeof(result));
  return result;
}

static inline uint32_t pack8to32ne(const uint8_t *buf){
  uint32_t result;
  memcpy(&result, buf, sizeof(result));
  return result;
}

static inline uint64_t pack8to64ne(const uint8_t *buf){
  uint64_t result;
  memcpy(&result, buf, sizeof(result));
  return result;
}

static inline size_t unpack16to8ne(uint16_t v, uint8_t *buf){
  memcpy(buf, &v, sizeof(v));
  return sizeof(v);
}

static inline size_t unpack32to8ne(uint32_t v, uint8_t *buf){
  memcpy(buf, &v, sizeof(v));
  return sizeof(v);
}

static inline size_t unpack64to8ne(uint64_t v, uint8_t *buf){
  memcpy(buf, &v, sizeof(v));
  return sizeof(v);
}

#define pack8to16be     pack8to16fe
#define pack8to32be     pack8to32fe
#define pack8to64be     pack8to64fe
#define unpack16to8be   unpack16to8fe
#define unpack32to8be   unpack32to8fe
#define unpack64to8be   unpack64to8fe

#define pack8to16le     pack8to16ne
#define pack8to32le     pack8to32ne
#define pack8to64le     pack8to64ne
#define unpack16to8le   unpack16to8ne
#define unpack32to8le   unpack32to8ne
#define unpack64to8le   unpack64to8ne

#elif BYTE_ORDER == BIG_ENDIAN
#error "Functions for this byte order are not implemented yet"
#else
#error "Endianness not defined"
#endif

#endif /* PACK_UNPACK_H_ */
