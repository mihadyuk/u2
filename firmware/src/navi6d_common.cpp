/*
 * Some functions was moved to this file to reduce copypasta size
 * between test and main code
 */



void Navi6dWrapper::read_settings(void) {

  param_registry.valueSearch("SINS_en_gnss",    &en_gnss);
  param_registry.valueSearch("SINS_en_odo",     &en_odo);
  param_registry.valueSearch("SINS_en_baro",    &en_baro);
  param_registry.valueSearch("SINS_en_euler",   &en_euler);
  param_registry.valueSearch("SINS_en_mag",     &en_mag);
  param_registry.valueSearch("SINS_en_nonhol",  &en_nonhol);
  param_registry.valueSearch("SINS_en_zihr",    &en_zihr);
  param_registry.valueSearch("SINS_en_gnss_v",  &en_gnss_v);
  param_registry.valueSearch("SINS_en_zupt",    &en_zupt);

  param_registry.valueSearch("SINS_R_ne_sns",   &R_ne_sns);
  param_registry.valueSearch("SINS_R_d_sns",    &R_d_sns);
  param_registry.valueSearch("SINS_R_v_n_sns",  &R_v_n_sns);
  param_registry.valueSearch("SINS_R_odo",      &R_odo);
  param_registry.valueSearch("SINS_R_nonhol",   &R_nonhol);
  param_registry.valueSearch("SINS_R_baro",     &R_baro);
  param_registry.valueSearch("SINS_R_mag",      &R_mag);
  param_registry.valueSearch("SINS_R_euler",    &R_euler);
  param_registry.valueSearch("SINS_R_zihr",     &R_zihr);

  param_registry.valueSearch("SINS_Qm_acc",     &Qm_acc);
  param_registry.valueSearch("SINS_Qm_gyr",     &Qm_gyr);
  param_registry.valueSearch("SINS_Qm_acc_x",   &Qm_acc_x);
  param_registry.valueSearch("SINS_Qm_acc_y",   &Qm_acc_y);
  param_registry.valueSearch("SINS_Qm_acc_z",   &Qm_acc_z);
  param_registry.valueSearch("SINS_Qm_gyr_bias",&Qm_gyr_bias);

  param_registry.valueSearch("SINS_eu_vh_roll", &eu_vh_roll);
  param_registry.valueSearch("SINS_eu_vh_pitch",&eu_vh_pitch);
  param_registry.valueSearch("SINS_eu_vh_yaw",  &eu_vh_yaw);

  param_registry.valueSearch("SINS_acc_bias_x", &acc_bias_x);
  param_registry.valueSearch("SINS_acc_bias_y", &acc_bias_y);
  param_registry.valueSearch("SINS_acc_bias_z", &acc_bias_z);

  param_registry.valueSearch("SINS_gyr_bias_x", &gyr_bias_x);
  param_registry.valueSearch("SINS_gyr_bias_y", &gyr_bias_y);
  param_registry.valueSearch("SINS_gyr_bias_z", &gyr_bias_z);

  param_registry.valueSearch("SINS_acc_scale_x",&acc_scale_x);
  param_registry.valueSearch("SINS_acc_scale_y",&acc_scale_y);
  param_registry.valueSearch("SINS_acc_scale_z",&acc_scale_z);

  param_registry.valueSearch("SINS_gyr_scale_x",&gyr_scale_x);
  param_registry.valueSearch("SINS_gyr_scale_y",&gyr_scale_y);
  param_registry.valueSearch("SINS_gyr_scale_z",&gyr_scale_z);

  param_registry.valueSearch("SINS_eu_vh_roll", &eu_vh_roll);
  param_registry.valueSearch("SINS_eu_vh_pitch",&eu_vh_pitch);
  param_registry.valueSearch("SINS_eu_vh_yaw",  &eu_vh_yaw);

  param_registry.valueSearch("GLRT_acc_sigma",  &acc_sigma);
  param_registry.valueSearch("GLRT_gyr_sigma",  &gyr_sigma);
  param_registry.valueSearch("GLRT_gamma",      &gamma);
  param_registry.valueSearch("GLRT_samples",    &samples);

  param_registry.valueSearch("SINS_restart",    &restart);
}

/**
 *
 */
void Navi6dWrapper::sins_cold_start(void) {

  nav_sins.params.ref_params.eu_vh_base[0][0] = deg2rad(*eu_vh_roll);
  nav_sins.params.ref_params.eu_vh_base[1][0] = deg2rad(*eu_vh_pitch);
  nav_sins.params.ref_params.eu_vh_base[2][0] = deg2rad(*eu_vh_yaw);

  nav_sins.params.init_params.est_gyro_bias = true;
  //init_params.sigma_Pi[0][0] = 200; //initial position STD (m)
  nav_sins.params.init_params.sigma_Pi[3][0] = M_PI; //initial heading STD (rad)
  nav_sins.params.init_params.dT = this->dT_cache;
  nav_sins.params.init_params.rst_dT = 0.5;

  nav_sins.params.kalman_params.sigma_R[0][0] = *R_ne_sns; //ne_sns
  nav_sins.params.kalman_params.sigma_R[1][0] = *R_d_sns; //d_sns
  nav_sins.params.kalman_params.sigma_R[2][0] = *R_v_n_sns; //v_n_sns
  nav_sins.params.kalman_params.sigma_R[3][0] = *R_odo; //odo
  nav_sins.params.kalman_params.sigma_R[4][0] = *R_nonhol; //nonhol
  nav_sins.params.kalman_params.sigma_R[5][0] = *R_baro; //baro
  nav_sins.params.kalman_params.sigma_R[6][0] = *R_mag; //mag
  nav_sins.params.kalman_params.sigma_R[7][0] = *R_euler; //roll,pitch,yaw (rad)
  nav_sins.params.kalman_params.sigma_R[8][0] = *R_zihr; // zihr

  nav_sins.params.kalman_params.sigma_Qm[0][0] = *Qm_acc; //acc
  nav_sins.params.kalman_params.sigma_Qm[1][0] = *Qm_gyr; //gyr
  nav_sins.params.kalman_params.sigma_Qm[2][0] = *Qm_acc_x; //acc_x
  nav_sins.params.kalman_params.sigma_Qm[3][0] = *Qm_acc_y; //acc_y
  nav_sins.params.kalman_params.sigma_Qm[4][0] = *Qm_acc_z; //acc_z
  nav_sins.params.kalman_params.sigma_Qm[5][0] = *Qm_gyr_bias; //gyr_bias

  nav_sins.params.calib_params.ba[0][0] = *acc_bias_x;
  nav_sins.params.calib_params.ba[1][0] = *acc_bias_y;
  nav_sins.params.calib_params.ba[2][0] = *acc_bias_z;

  nav_sins.params.calib_params.bw[0][0] = *gyr_bias_x;
  nav_sins.params.calib_params.bw[1][0] = *gyr_bias_y;
  nav_sins.params.calib_params.bw[2][0] = *gyr_bias_z;

  nav_sins.params.calib_params.sa[0][0] = *acc_scale_x;
  nav_sins.params.calib_params.sa[1][0] = *acc_scale_y;
  nav_sins.params.calib_params.sa[2][0] = *acc_scale_z;

  nav_sins.params.calib_params.sw[0][0] = *gyr_scale_x;
  nav_sins.params.calib_params.sw[1][0] = *gyr_scale_y;
  nav_sins.params.calib_params.sw[2][0] = *gyr_scale_z;

  nav_sins.params.calib_params.bm[0][0] = -3.79611/1000;
  nav_sins.params.calib_params.bm[1][0] = 15.2098/1000;
  nav_sins.params.calib_params.bm[2][0] = -5.45266/1000;

  nav_sins.params.calib_params.m_s[0][0] = 0.916692;
  nav_sins.params.calib_params.m_s[1][0] = 0.912;
  nav_sins.params.calib_params.m_s[2][0] = 0.9896;

  nav_sins.params.calib_params.m_no[0][0] = -0.0031;
  nav_sins.params.calib_params.m_no[1][0] = 0.0078;
  nav_sins.params.calib_params.m_no[2][0] = 0.0018;

  CommandType<klmnfp> cmd;
  cmd.command = 4;
  cmd.param[0][0] = 0.941197195644872;
  cmd.param[1][0] = 0.481544438952965;
  cmd.param[2][0] = 252;
  cmd.param[8][0] = M_PI;

  nav_sins.command_executor(cmd);
}
/**
 *
 */
void Navi6dWrapper::prepare_data_gnss(gnss::gnss_data_t &gnss_data) {

  if ((*en_gnss == 1) && (gnss_data.fresh) && (gnss_data.fix > 0)) {
    nav_sins.sensor_data.r_sns[0][0] = deg2rad(gnss_data.latitude);
    nav_sins.sensor_data.r_sns[1][0] = deg2rad(gnss_data.longitude);
    nav_sins.sensor_data.r_sns[2][0] = gnss_data.altitude;
    nav_sins.sensor_flags.sns_r_en = true;
    nav_sins.sensor_flags.sns_h_en = true;
    if (*en_gnss_v == 1) {
      switch(gnss_data.speed_type) {
      case gnss::speed_t::SPEED_COURSE:
        nav_sins.sensor_data.v_sns[0][0] = gnss_data.speed * cos(deg2rad(gnss_data.course));
        nav_sins.sensor_data.v_sns[1][0] = gnss_data.speed * sin(deg2rad(gnss_data.course));
        nav_sins.sensor_data.v_sns[2][0] = gnss_data.course;
        nav_sins.sensor_flags.sns_v_n_en = true;
        nav_sins.sensor_flags.sns_v_e_en = true;
        nav_sins.sensor_flags.sns_v_d_en = false;
        break;
      case gnss::speed_t::VECTOR_3D:
      case gnss::speed_t::BOTH:
        for (size_t i=0; i<3; i++) {
          nav_sins.sensor_data.v_sns[i][0] = gnss_data.v[i];
        }
        nav_sins.sensor_flags.sns_v_n_en = true;
        nav_sins.sensor_flags.sns_v_e_en = true;
        nav_sins.sensor_flags.sns_v_d_en = true;
        break;
      default:
        nav_sins.sensor_flags.sns_v_n_en = false;
        nav_sins.sensor_flags.sns_v_e_en = false;
        nav_sins.sensor_flags.sns_v_d_en = false;
        break;
      }
    }
  }

  // Important! Must be set to false after data processing
  if (gnss_data.fresh) {
    gnss_data.fresh = false;
  }
}

/*
 *
 */
void Navi6dWrapper::prepare_data(const baro_data_t &baro,
                                 const odometer_data_t &odo,
                                 const marg_data_t &marg) {

  nav_sins.params.ctrl_params.use_odo = *en_odo;
  nav_sins.params.ctrl_params.use_zupt = false;
  nav_sins.params.ctrl_params.use_zihr = false;
  nav_sins.params.ctrl_params.use_mag_vec = false;
  nav_sins.params.ctrl_params.use_baro_alt = *en_baro;
  nav_sins.params.ctrl_params.use_nonhol_y = *en_nonhol;
  nav_sins.params.ctrl_params.use_nonhol_z = *en_nonhol;
  nav_sins.params.ctrl_params.use_yaw = false;
  nav_sins.params.ctrl_params.use_mag_course = false;
  nav_sins.sensor_flags.odo_en = odo.fresh;
  nav_sins.sensor_flags.mag_en = false;
  nav_sins.sensor_flags.alt_b_en = true;

  nav_sins.sensor_data.alt_b[0][0] = baro.alt;
  nav_sins.sensor_data.v_odo[0][0] = odo.speed;
  for(size_t i=0; i<3; i++) {
    nav_sins.sensor_data.fb[i][0] = marg.acc[i];
    nav_sins.sensor_data.wb[i][0] = marg.gyr[i];
    nav_sins.sensor_data.mb[i][0] = marg.mag[i];
  }
}
