#ifndef MAVENCODE_H_
#define MAVENCODE_H_

#ifdef __cplusplus
extern "C" {
#endif
  uint16_t mavlink_encode(uint8_t msgid, mavlink_message_t* msg, const void* mavlink_struct);
#ifdef __cplusplus
}
#endif

#endif /* MAVENCODE_H_ */
