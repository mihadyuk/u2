#ifndef CONTROL_STATE_SWITCHER_HPP_
#define CONTROL_STATE_SWITCHER_HPP_

#include "mav_mail.hpp"
#include "subscribe_link.hpp"

namespace control {

/**
 *
 */
class StateSwitcher {
public:
  StateSwitcher(mavlink_system_info_t &sysinfo);
  void start(void);
  void update(void);
  void stop(void);
private:
  chibios_rt::Mailbox<mavMail*, 1> command_mailbox;
  SubscribeLink command_long_link;
  mavlink_system_info_t &sysinfo;
};

} // namespace

#endif /* CONTROL_STATE_SWITCHER_HPP_ */
