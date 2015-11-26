#ifndef MTKGPS_HPP_
#define MTKGPS_HPP_

#include "gnss_receiver.hpp"
#include "proto_nmea.hpp"

namespace gnss {

/**
 *
 */
class mtkgps : public GNSSReceiver {
public:
  mtkgps(SerialDriver *sdp, uint32_t start_baudrate,
                            uint32_t working_baudrate);
  void start(void);
private:
  NmeaProto nmea_parser;
  static THD_FUNCTION(nmeaRxThread, arg);
  void update_settings(void);
  void configure(void);
  void ggarmc2mavlink(const nmea_gga_t &gga, const nmea_rmc_t &rmc);
  const uint32_t *fix_period = nullptr;
  uint32_t fix_period_cache = 200;
  gnss_data_t cache;
};

} // namespace

#endif /* MTKGPS_HPP_ */
