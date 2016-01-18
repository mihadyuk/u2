#ifndef TIME_KEEPER_HPP_
#define TIME_KEEPER_HPP_

#include "gnss_receiver.hpp"

/* Use this constant if correct time value can not be determined */
#define TIME_INVALID    0

/**
 *
 */
class TimeKeeper {
public:
  TimeKeeper(gnss::GNSSReceiver &GNSS);
  void start(void);
  void stop(void);
  int format_time(char *str, size_t len);
  static void PPS_ISR_I(void);
  static int64_t utc(void);
  static void utc(int64_t t);
  static int format_time(int64_t time, char *str, size_t len);
private:
  THD_WORKING_AREA(TimekeeperThreadWA, 512);
  static THD_FUNCTION(TimekeeperThread, arg);
  thread_t *worker = nullptr;
  gnss::GNSSReceiver &GNSS;
  static bool ready;
  static bool time_verified;
};

#include "cli.hpp"
thread_t* date_clicmd(int argc, const char * const * argv, BaseChannel *bchnp);

#endif /* TIME_KEEPER_HPP_ */
