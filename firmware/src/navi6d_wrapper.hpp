#ifndef NAVI6D_WRAPPER_HPP_
#define NAVI6D_WRAPPER_HPP_

#include "baro_data.hpp"
#include "speedometer_data.hpp"
#include "acs_input.hpp"
#include "gnss_receiver.hpp"
#include "marg_data.hpp"

/**
 *
 */
class Navi6dWrapper {
public:
  Navi6dWrapper(ACSInput &acs_in, gnss::GNSSReceiver &GNSS);
  void update(const baro_data_t &abs_press,
              const speedometer_data_t &speed,
              const marg_data_t &marg);
  void start(float dT);
  void stop(void);
private:
  void navi2acs(void);
  void prepare_data(const baro_data_t &abs_press,
                    const speedometer_data_t &speed,
                    const marg_data_t &marg);
  void prepare_gnss(const speedometer_data_t &speed);
  void reload_settings(void);
  bool ready = false;
  ACSInput &acs_in;
  gnss::GNSSReceiver &GNSS;
  gnss::gnss_data_t gnss_data;
  const uint32_t *gnss_enable = nullptr;
  const uint32_t *odo_enable  = nullptr;
  const uint32_t *baro_enable = nullptr;

  const float *acc_sigma = nullptr;
  const float *gyr_sigma = nullptr;
  const float *gamma = nullptr;
  const float *sigma_R0 = nullptr;
  const float *sigma_R1 = nullptr;
  const float *sigma_R2 = nullptr;
  const float *sigma_R3 = nullptr;
  const float *sigma_R4 = nullptr;
  const float *sigma_R5 = nullptr;
  const float *sigma_R6 = nullptr;
  const float *sigma_Qm0 = nullptr;
  const float *sigma_Qm1 = nullptr;
  const float *sigma_Qm2 = nullptr;
  const float *sigma_Qm3 = nullptr;
  const float *sigma_Qm4 = nullptr;
  const float *sigma_Qm5 = nullptr;
  const float *eu_vh_roll = nullptr;
  const float *eu_vh_pitch = nullptr;
  const float *eu_vh_yaw = nullptr;
  const uint32_t *samples = nullptr;
};

#endif /* NAVI6D_WRAPPER_HPP_ */
