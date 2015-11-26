#ifndef MTKGPS_HPP_
#define MTKGPS_HPP_

#include "nmea_generic.hpp"
#include "proto_nmea.hpp"

namespace gnss {

/**
 *
 */
class mtkgps : public nmea_generic {
public:
  mtkgps(SerialDriver *sdp, uint32_t start_baudrate,
                            uint32_t working_baudrate);
  void start(void);
protected:
  void load_params(void);
private:
  void configure(void);
  const uint32_t *fix_period = nullptr;
  uint32_t fix_period_cache = 200;
};

} // namespace

#endif /* MTKGPS_HPP_ */
