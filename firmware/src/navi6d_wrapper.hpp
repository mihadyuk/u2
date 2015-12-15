#ifndef NAVI6D_WRAPPER_HPP_
#define NAVI6D_WRAPPER_HPP_

#include "baro_data.hpp"
#include "odometer_data.hpp"
#include "acs_input.hpp"
#include "gnss_receiver.hpp"
#include "marg_data.hpp"

/**
 *
 */
class Navi6dWrapper {
public:
  Navi6dWrapper(ACSInput &acs_in, gnss::GNSSReceiver &GNSS);
  void update(const baro_data_t &baro,
              const odometer_data_t &odo,
              const marg_data_t &marg);
  void start(void);
  void stop(void);
private:
  void read_settings(void);
  void sins_cold_start(void);
  void navi2acs(void);
  void navi2mavlink(void);
  void debug2mavlink(float dT);
  void prepare_data(const baro_data_t &baro,
                    const odometer_data_t &odo,
                    const marg_data_t &marg);
  void prepare_data_gnss(gnss::gnss_data_t &gnss_data);
  void reload_settings(void);
  void start_time_measurement(void);
  void stop_time_measurement(float dT);
  time_measurement_t tmeas;
  size_t time_overrun_cnt;
  float time_meas_decimator = 0;
  bool ready = false;
  float dT_cache = 0.01;
  float debug_vect_decimator = 0;
  float debug_decimator = 0;
  ACSInput &acs_in;
  gnss::GNSSReceiver &GNSS;
  gnss::gnss_data_t gps;
  chibios_rt::EvtListener gnss_evl;

  const uint32_t *T_debug      = nullptr;
  const uint32_t *T_debug_vect = nullptr;

  const uint32_t *en_gnss    = nullptr;
  const uint32_t *en_baro    = nullptr;
  const uint32_t *en_odo     = nullptr;
  const uint32_t *en_nhl_y   = nullptr;
  const uint32_t *en_nhl_z   = nullptr;
  const uint32_t *en_roll    = nullptr;
  const uint32_t *en_pitch   = nullptr;
  const uint32_t *en_yaw     = nullptr;
  const uint32_t *en_mg_v    = nullptr;
  const uint32_t *en_mg_yaw  = nullptr;
  const uint32_t *zupt_src   = nullptr;

  const float *acc_sigma    = nullptr;
  const float *gyr_sigma    = nullptr;
  const float *gamma        = nullptr;
  const float *R_pos_sns    = nullptr;
  const float *R_vel_sns    = nullptr;
  const float *R_odo        = nullptr;
  const float *R_nhl_y      = nullptr;
  const float *R_nhl_z      = nullptr;
  const float *R_baro       = nullptr;
  const float *R_mag        = nullptr;
  const float *R_euler      = nullptr;
  const float *R_mag_yaw    = nullptr;
  const float *R_v_nav_st   = nullptr;
  const float *R_v_veh_st   = nullptr;
  const float *R_yaw_st     = nullptr;

  const float *init_lat     = nullptr;
  const float *init_lon     = nullptr;
  const float *init_alt     = nullptr;
  const float *init_yaw     = nullptr;
  const float *c_alg_t      = nullptr;
  const float *f_alg_t      = nullptr;
  const float *Qm_acc       = nullptr;
  const float *Qm_gyr       = nullptr;
  const float *Qm_acc_x     = nullptr;
  const float *Qm_acc_y     = nullptr;
  const float *Qm_acc_z     = nullptr;
  const float *Qm_gyr_bias  = nullptr;
  const float *eu_vh_roll   = nullptr;
  const float *eu_vh_pitch  = nullptr;
  const float *eu_vh_yaw    = nullptr;

  const float *P_ned        = nullptr;
  const float *P_acc_b      = nullptr;
  const float *P_gyr_b      = nullptr;

  const float *B_acc_b      = nullptr;
  const float *B_gyr_b      = nullptr;

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

  const float *gyr_nort_0  = nullptr;
  const float *gyr_nort_1  = nullptr;
  const float *gyr_nort_2  = nullptr;
  const float *gyr_nort_3  = nullptr;
  const float *gyr_nort_4  = nullptr;
  const float *gyr_nort_5  = nullptr;
  const float *acc_nort_0  = nullptr;
  const float *acc_nort_1  = nullptr;
  const float *acc_nort_2  = nullptr;
  const float *acc_nort_3  = nullptr;
  const float *acc_nort_4  = nullptr;
  const float *acc_nort_5  = nullptr;


  const uint32_t *samples   = nullptr;

  const uint32_t *restart   = nullptr;
  uint32_t restart_cache = 0;
};

#endif /* NAVI6D_WRAPPER_HPP_ */
