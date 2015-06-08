#ifndef NAVI6D_WRAPPER_HPP_
#define NAVI6D_WRAPPER_HPP_

#include "baro_data.hpp"
#include "speedometer_data.hpp"
#include "acs_input.hpp"
#include "gps_data.hpp"
#include "marg_data.hpp"

/**
 *
 */
class Navi6dWrapper {
public:
  Navi6dWrapper(ACSInput &acs_in);
  void update(const gps_data_t &gps_data,
              const baro_data_t &abs_press,
              const speedometer_data_t &speed,
              const marg_data_t &marg);
  void start(float dT);
  void stop(void);
private:
  void navi2acs(void);
  void prepare_data(const gps_data_t &gps_data,
                    const baro_data_t &abs_press,
                    const speedometer_data_t &speed,
                    const marg_data_t &marg);
  void reload_settings(void);
  bool ready = false;
  chibios_rt::EvtListener el;
  ACSInput &acs_in;
  const uint32_t *gnss_block = nullptr;
  const uint32_t *odo_block  = nullptr;
  const uint32_t *baro_block = nullptr;
};

#endif /* NAVI6D_WRAPPER_HPP_ */
