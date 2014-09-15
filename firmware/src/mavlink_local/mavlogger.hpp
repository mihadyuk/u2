#ifndef MAVLOGGER_HPP_
#define MAVLOGGER_HPP_

#include "mavmail.hpp"

class MavLogger {
public:
  MavLogger(void);
  msg_t post(mavMail* msg);
  void start(void);
  void stop(void);
private:
  thread_t *worker = NULL;
  chibios_rt::Mailbox<mavMail*, 8> mb;
  size_t drop_cnt = 0;
  bool ready = false;
};

#endif /* MAVLOGGER_HPP_ */
