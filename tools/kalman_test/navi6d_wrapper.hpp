#ifndef NAVI6D_WRAPPER_HPP_
#define NAVI6D_WRAPPER_HPP_

#include <cstring>
#include <ctime>

#include "../../firmware/src/sensors/baro_data.hpp"
#include "../../firmware/src/sensors/odometer_data.hpp"
#include "../../firmware/src/sensors/gnss/gnss_data.hpp"
#include "../../firmware/src/sensors/marg_data.hpp"

/**
 *
 */
class Navi6dWrapper {
public:
  Navi6dWrapper(void);
  void update(const mavlink_navi6d_debug_input_t &test,
              const mavlink_navi6d_debug_output_t &ref);
  void start(void);
  void stop(void);
private:

  void read_settings(void);
  void sins_cold_start(void);
  void navi2acs(void);
  void navi2mavlink(void);
  void debug2mavlink(void);
  void prepare_data(const baro_data_t &baro,
                    const odometer_data_t &odo,
                    const marg_data_t &marg,
                    gnss::gnss_data_t &gps);
  void prepare_data_gnss(gnss::gnss_data_t &gnss_data);
  void reload_settings(void);
  bool ready = false;
  float dT_cache = 0.01;

  const uint32_t *en_gnss   = nullptr;
  const uint32_t *en_odo    = nullptr;
  const uint32_t *en_baro   = nullptr;
  const uint32_t *en_euler  = nullptr;
  const uint32_t *en_nonhol = nullptr;
  const uint32_t *en_mag    = nullptr;
  const uint32_t *en_zihr   = nullptr;
  const uint32_t *en_gnss_v = nullptr;
  const uint32_t *en_zupt   = nullptr;

  const float *acc_sigma    = nullptr;
  const float *gyr_sigma    = nullptr;
  const float *gamma        = nullptr;
  const float *R_ne_sns     = nullptr;
  const float *R_d_sns      = nullptr;
  const float *R_v_n_sns    = nullptr;
  const float *R_odo        = nullptr;
  const float *R_nonhol     = nullptr;
  const float *R_baro       = nullptr;
  const float *R_mag        = nullptr;
  const float *R_euler      = nullptr;
  const float *R_zihr       = nullptr;
  const float *Qm_acc       = nullptr;
  const float *Qm_gyr       = nullptr;
  const float *Qm_acc_x     = nullptr;
  const float *Qm_acc_y     = nullptr;
  const float *Qm_acc_z     = nullptr;
  const float *Qm_gyr_bias  = nullptr;
  const float *eu_vh_roll   = nullptr;
  const float *eu_vh_pitch  = nullptr;
  const float *eu_vh_yaw    = nullptr;

  const float *acc_bias_x   = nullptr;
  const float *acc_bias_y   = nullptr;
  const float *acc_bias_z   = nullptr;
  const float *gyr_bias_x   = nullptr;
  const float *gyr_bias_y   = nullptr;
  const float *gyr_bias_z   = nullptr;

  const float *acc_scale_x  = nullptr;
  const float *acc_scale_y  = nullptr;
  const float *acc_scale_z  = nullptr;
  const float *gyr_scale_x  = nullptr;
  const float *gyr_scale_y  = nullptr;
  const float *gyr_scale_z  = nullptr;

  const uint32_t *samples   = nullptr;

  const uint32_t *restart   = nullptr;
  uint32_t restart_cache = 0;
};

#endif /* NAVI6D_WRAPPER_HPP_ */
