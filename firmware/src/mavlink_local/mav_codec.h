#ifndef MAV_CODEC_H_
#define MAV_CODEC_H_

#ifdef __cplusplus
extern "C" {
#endif
  uint16_t mavlink_encode(uint8_t msgid, MAV_COMPONENT compid,
                          mavlink_message_t* msg, const void* mavlink_struct);
  size_t mavlink_decode_memsize(const mavlink_message_t* msg);
  void mavlink_decode(const mavlink_message_t* msg, void* mavlink_struct);
#ifdef __cplusplus
}
#endif

#endif /* MAV_CODEC_H_ */
