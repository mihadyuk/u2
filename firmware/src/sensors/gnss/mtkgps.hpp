#ifndef MTKGPS_HPP_
#define MTKGPS_HPP_

#include "nmeageneric.hpp"

namespace gnss {

/**
 *
 */
class mtkgps : public nmeageneric {
public:
  mtkgps(SerialDriver *sdp, uint32_t start_baudrate,
                            uint32_t working_baudrate);
private:
  void configure(void);
  void update_settings(void);
  void load_params(void);
  const uint32_t *fix_period = nullptr;
  uint32_t fix_period_cache = 200;
};

} // namespace

#endif /* MTKGPS_HPP_ */
