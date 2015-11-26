#ifndef NMEA_GENERIC_HPP_
#define NMEA_GENERIC_HPP_

#include "gnss_receiver.hpp"
#include "proto_nmea.hpp"

namespace gnss {

/**
 *
 */
class nmea_generic : public GNSSReceiver {
public:
  nmea_generic(SerialDriver *sdp, uint32_t start_baudrate,
                                  uint32_t working_baudrate);
  void start(void);
protected:
  virtual void load_params(void){return;}
  void start_impl(void);
private:
  NmeaProto nmea_parser;
  static THD_FUNCTION(nmeaRxThread, arg);
  void ggarmc2mavlink(const nmea_gga_t &gga, const nmea_rmc_t &rmc);
  void gnss_unpack(const nmea_gga_t &gga, const nmea_rmc_t &rmc,
                         gnss_data_t *result);
};

} // namespace

#endif /* NMEA_GENERIC_HPP_ */
