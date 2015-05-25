#include "navigator.hpp"
static Navigator<double,15,9> Nav;

static InitParams<double> init_params;
static CalibParams<double> calib_params;
static KalmanParams<double> kalman_params;
static RefParams<double> ref_params;

static SensorData<double> sensor_data;
static KalmanFlags sensor_flags;
static navigator_mode_t mode;
static uint8_t start_prev;
static uint8_t cmd;

for (int i = 0; i<3;i++){
  ref_params.eu_hb[i][0] = eu_hb_in[i];
  ref_params.eu_vh_base[i][0] = eu_vh_base_in[i];
}


//interface
for (int i = 0; i<3;i++){
  sensor_data.r_sns[i][0] = r_sns_in[i];
  sensor_data.v_sns[i][0] = v_sns_in[i];
  sensor_data.v_odo[i][0] = 0.0;
  sensor_data.ang[i][0] = 0.0;
  sensor_data.fb[i][0] = Acc[i];
  sensor_data.wb[i][0] = Gyr[i];
  calib_params.ba_sat[i][0] = ba_sat_in[i];
  calib_params.bw_sat[i][0] = bw_sat_in[i];
  calib_params.sa_sat[i][0] = 0.0;
  calib_params.sw_sat[i][0] = 0.0;
  init_params.r_init[i][0] = r_init_in[i];
  init_params.v_init[i][0] = v_init_in[i];
  init_params.qnv_init[i][0] = qnv_init_in[i];
  init_params.eu_nv_init[i][0] = eu_nv_init_in[i];
}
init_params.qnv_init[3][0] = qnv_init_in[3];
init_params.init_time = 5.0;

sensor_data.mb[0][0] = 1.0;
sensor_data.mb[1][0] = 0.0;
sensor_data.mb[2][0] = 0.0;
sensor_data.pps = pps_in[0];

calib_params.ga_sat = 0.0;
calib_params.gw_sat = 0.0;
calib_params.alpha = 0.02;

sensor_data.alt_b[0][0] = 0.0;

for (int i = 0; i<9;i++){
  kalman_params.R[i][0] = 1.0;
}
for (int i = 0; i<3;i++){
  kalman_params.R[i][0] = R_in[i];
}

for (int i = 0; i<10;i++){
  kalman_params.Qm[i][0] = 1.0;
}

kalman_params.Qm[0][0] = Qm_in[0];
kalman_params.Qm[1][0] = Qm_in[1];
kalman_params.Qm[2][0] = Qm_in[2];
kalman_params.Qm[3][0] = Qm_in[2];
kalman_params.Qm[4][0] = Qm_in[2];
kalman_params.Qm[5][0] = Qm_in[3];

for (int i = 0; i<10;i++){
  init_params.Pi[i][0] = 0.0;
}
init_params.Pi[0][0] = 15.0; //r
init_params.Pi[1][0] = 0.5; //v
init_params.Pi[2][0] = 0.5; //tilt
init_params.Pi[3][0] = 50.0; //head
init_params.Pi[4][0] = 0.0000000001; //ba
init_params.Pi[5][0] = 0.0000000001; //bw


init_params.dT = dT_in[0];
init_params.rst_cnt = rst_cnt_in[0];
init_params.alpha = 0.03;

sensor_flags.sns_r_en = sns_en[0];
sensor_flags.sns_h_en = sns_en[1];
sensor_flags.sns_v_n_en = sns_en[2];
sensor_flags.sns_v_e_en = sns_en[3];
sensor_flags.sns_v_d_en = sns_en[4];

cmd = start[0] - start_prev;
start_pulse_out[0] = cmd;
start_prev = start[0];


///Initialize SINS
Nav.set_init_params(init_params);
Nav.set_calib_params(calib_params);
Nav.set_kalman_params(kalman_params);
Nav.set_ref_params(ref_params);
Nav.command_executor(cmd);
Nav.run(sensor_data, sensor_flags);


qnv_out[3] = Nav.navi_data.qnv[3][0];
for (int i = 0; i<3; i++){
  qnv_out[i] = Nav.navi_data.qnv[i][0];
  r_out[i] = Nav.navi_data.r[i][0];
  v_out[i] = Nav.navi_data.v[i][0];
  eu_out[i] = Nav.navi_data.eu_nv[i][0];
  a_b_out[i] = Nav.navi_data.a_b[i][0];
  w_b_out[i] = Nav.navi_data.w_b[i][0];
  acc_free_out[i] = Nav.navi_data.free_acc[i][0];
}
status_out[0] = Nav.navi_data.status;