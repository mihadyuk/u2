#ifndef TIME_KEEPER_HPP_
#define TIME_KEEPER_HPP_

/* Always use this constant if correct time value can not be determined */
#define TIME_INVALID    0

/**
 *
 */
class TimeKeeper {
public:
  TimeKeeper(void);
  void start(void);
  void stop(void);
  static int64_t utc(void);
  static void utc(int64_t t);
  int format_time(char *str, size_t len);
  static int format_time(int64_t time, char *str, size_t len);
  static void normal_pps_isr(EXTDriver *extp, expchannel_t channel);
  static void PPS_ISR(void);

private:
  friend THD_FUNCTION(TimekeeperThread, arg);
  thread_t *worker = nullptr;
  static bool ready;
};

#include "cli.hpp"
thread_t* date_clicmd(int argc, const char * const * argv, SerialDriver *sdp);

#endif /* TIME_KEEPER_HPP_ */
