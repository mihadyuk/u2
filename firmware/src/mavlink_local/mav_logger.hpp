#ifndef MAV_LOGGER_HPP_
#define MAV_LOGGER_HPP_

#include "mav_mail.hpp"

/*
 *
 */
class MavLogger : public chibios_rt::BaseStaticThread<2048> {
public:
  msg_t post(mavMail* msg);
  void stop(void);

private:
  msg_t main(void);
  size_t drop_cnt = 0;
  bool ready = false;
  chibios_rt::Mailbox<mavMail*, 12> logwriter_mb;
};

#endif /* MAV_LOGGER_HPP_ */
