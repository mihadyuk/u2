#ifndef TIME_KEEPER_H_
#define TIME_KEEPER_H_

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

private:
  Thread *worker;
  Thread *soft_pps;
  static bool ready;
};

#include "cli.hpp"
Thread* date_clicmd(int argc, const char * const * argv, SerialDriver *sdp);

#endif /* TIME_KEEPER_H_ */
