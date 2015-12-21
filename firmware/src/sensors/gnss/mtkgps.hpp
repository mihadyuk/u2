#ifndef MTKGPS_HPP_
#define MTKGPS_HPP_

#include "generic_nmea.hpp"

namespace gnss {

/**
 *
 */
class mtkgps : public GenericNMEA {
public:
  mtkgps(SerialDriver *sdp, uint32_t start_baudrate,
                            uint32_t working_baudrate);
private:
  void configure(void);
  void update_settings(void);
  void load_params(void);
  const uint32_t *fix_period = nullptr;
  uint32_t fix_period_cache = 200;
  char txbuf[80];
};

} // namespace

#endif /* MTKGPS_HPP_ */
