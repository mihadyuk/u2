#ifndef MOD_TELEM_HPP_
#define MOD_TELEM_HPP_

#include "mav_channel_serial.hpp"

/*
 *
 */
class ModTelem : public chibios_rt::BaseStaticThread<256> {
public:
  void stop(void);
private:
  void main(void);
  void start_impl(void);
  void stop_impl(void);
  void push(uint8_t msgid, const void* mavlink_struct);
  size_t drop_cnt = 0;
};

#endif /* MOD_TELEM_HPP_ */
