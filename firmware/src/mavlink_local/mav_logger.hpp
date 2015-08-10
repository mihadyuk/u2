#ifndef MAV_LOGGER_HPP_
#define MAV_LOGGER_HPP_

#include "mav_mail.hpp"
#include "multi_buffer.hpp"
#include "ff.h"

/*
 *
 */
class MavLogger : public chibios_rt::BaseStaticThread<2048> {
public:
  msg_t write(mavMail* msg);
  void stop(void);

private:
  void main(void);
  FRESULT append_log(mavMail *mail, bool *fresh_data);
  size_t drop_cnt = 0;
  bool ready = false;
  chibios_rt::Mailbox<mavMail*, 24> logwriter_mb;
  MultiBufferAccumulator<uint8_t, 8192, 2> double_buf;
  FIL log_file;
};

#endif /* MAV_LOGGER_HPP_ */
