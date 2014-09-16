#ifndef MAV_ENCODE_H_
#define MAV_ENCODE_H_

#ifdef __cplusplus
extern "C" {
#endif
  uint16_t mavlink_encode(uint8_t msgid, mavlink_message_t* msg, const void* mavlink_struct);
#ifdef __cplusplus
}
#endif

#endif /* MAV_ENCODE_H_ */
