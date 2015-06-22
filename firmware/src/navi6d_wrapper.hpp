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
  const float *R_ne_sns = nullptr;
  const float *R_d_sns = nullptr;
  const float *R_v_n_sns = nullptr;
  const float *R_odo = nullptr;
  const float *R_nonhol = nullptr;
  const float *R_baro = nullptr;
  const float *R_mag = nullptr;
  const float *Qm_acc = nullptr;
  const float *Qm_gyr = nullptr;
  const float *Qm_acc_x = nullptr;
  const float *Qm_acc_y = nullptr;
  const float *Qm_acc_z = nullptr;
  const float *Qm_gyr_bias = nullptr;
  const float *eu_vh_roll = nullptr;
  const float *eu_vh_pitch = nullptr;
  const float *eu_vh_yaw = nullptr;
  const uint32_t *samples = nullptr;
};

#endif /* NAVI6D_WRAPPER_HPP_ */
