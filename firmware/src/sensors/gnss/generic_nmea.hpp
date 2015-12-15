#ifndef GENERIC_NMEA_HPP_
#define GENERIC_NMEA_HPP_

#include "gnss_receiver.hpp"
#include "proto_nmea.hpp"

namespace gnss {

/**
 *
 */
class GenericNMEA : public GNSSReceiver {
public:
  GenericNMEA(SerialDriver *sdp, uint32_t start_baudrate,
                                  uint32_t working_baudrate);
protected:
  virtual void configure(void);
  virtual void load_params(void){return;}
  virtual void update_settings(void){return;}
  virtual void subscribe_inject(void){return;}
  virtual void release_inject(void){return;}
  virtual void inject(void){return;}
  void start_impl(void);
  ProtoNmea parser;
private:
  static THD_FUNCTION(nmeaRxThread, arg);
  void ggarmc2mavlink(const nmea_gga_t &gga, const nmea_rmc_t &rmc);
  void gnss_unpack(const nmea_gga_t &gga, const nmea_rmc_t &rmc,
                         gnss_data_t *result);
  void dispatch_data(const nmea_gga_t &gga, const nmea_rmc_t &rmc);
};

} // namespace

#endif /* GENERIC_NMEA_HPP_ */
