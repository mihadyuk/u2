#ifndef CRC_H_
#define CRC_H_

#ifdef __cplusplus
extern "C" {
#endif
  uint32_t crc32(const uint8_t *buf, size_t len);
  uint8_t crc8(const uint8_t *buf, size_t len);
#ifdef __cplusplus
}
#endif

#endif /* CRC_H_ */
